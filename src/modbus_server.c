#include "modbus_server.h"
#include "io_handler.h"

#include "socket.h"
#include "wizchip_conf.h"

#include <string.h>
#include <stdio.h>

// Modbus TCP buffers
static uint8_t g_rx_buffer[MODBUS_TCP_BUFFER_SIZE];
static uint8_t g_tx_buffer[MODBUS_TCP_BUFFER_SIZE];

// Forward declarations
static int16_t modbus_process_request(uint8_t* request, uint16_t request_len, uint8_t* response);
static int16_t modbus_read_coils(uint16_t start_addr, uint16_t quantity, uint8_t* response);
static int16_t modbus_read_discrete_inputs(uint16_t start_addr, uint16_t quantity, uint8_t* response);
static int16_t modbus_read_holding_registers(uint16_t start_addr, uint16_t quantity, uint8_t* response);
static int16_t modbus_write_single_coil(uint16_t addr, uint16_t value, uint8_t* response);
static int16_t modbus_write_multiple_coils(uint16_t start_addr, uint16_t quantity, uint8_t* data, uint8_t* response);
static void modbus_build_exception(uint8_t function_code, uint8_t exception_code, uint8_t* response);

void modbus_server_init(void)
{
    memset(g_rx_buffer, 0, sizeof(g_rx_buffer));
    memset(g_tx_buffer, 0, sizeof(g_tx_buffer));
}

bool modbus_server_run(void)
{
    uint8_t sock_status;
    int16_t recv_len;
    int16_t resp_len;

    sock_status = getSn_SR(MODBUS_SOCKET);

    switch (sock_status) {
        case SOCK_ESTABLISHED:
            // Check if data is available
            recv_len = getSn_RX_RSR(MODBUS_SOCKET);
            if (recv_len > 0) {
                // Limit receive length to buffer size
                if (recv_len > MODBUS_TCP_BUFFER_SIZE) {
                    recv_len = MODBUS_TCP_BUFFER_SIZE;
                }

                // Receive data
                recv_len = recv(MODBUS_SOCKET, g_rx_buffer, recv_len);
                if (recv_len > 0) {
                    // Process Modbus request
                    resp_len = modbus_process_request(g_rx_buffer, recv_len, g_tx_buffer);
                    if (resp_len > 0) {
                        // Send response
                        send(MODBUS_SOCKET, g_tx_buffer, resp_len);
                    }
                    return true;
                }
            }
            break;

        case SOCK_CLOSE_WAIT:
            // Client closed connection, disconnect
            disconnect(MODBUS_SOCKET);
            break;

        case SOCK_CLOSED:
            // Open socket for listening
            if (socket(MODBUS_SOCKET, Sn_MR_TCP, MODBUS_TCP_PORT, 0x00) == MODBUS_SOCKET) {
                listen(MODBUS_SOCKET);
            }
            break;

        case SOCK_INIT:
            // Socket initialized, start listening
            listen(MODBUS_SOCKET);
            break;

        case SOCK_LISTEN:
            // Waiting for connection
            break;

        default:
            break;
    }

    return false;
}

uint8_t modbus_server_get_socket_state(void)
{
    return getSn_SR(MODBUS_SOCKET);
}

static int16_t modbus_process_request(uint8_t* request, uint16_t request_len, uint8_t* response)
{
    // Validate minimum frame length (MBAP header + function code)
    if (request_len < MODBUS_MBAP_HEADER_SIZE + 1) {
        return -1;
    }

    // Parse MBAP header
    uint16_t transaction_id = (request[0] << 8) | request[1];
    uint16_t protocol_id = (request[2] << 8) | request[3];
    uint16_t length = (request[4] << 8) | request[5];
    uint8_t unit_id = request[6];

    // Validate protocol ID (must be 0x0000 for Modbus TCP)
    if (protocol_id != 0x0000) {
        return -1;
    }

    // Get function code and data
    uint8_t* pdu = &request[MODBUS_MBAP_HEADER_SIZE];
    uint8_t function_code = pdu[0];

    // Response PDU buffer (after MBAP header)
    uint8_t* resp_pdu = &response[MODBUS_MBAP_HEADER_SIZE];
    int16_t resp_pdu_len = 0;

    // Process based on function code
    switch (function_code) {
        case MODBUS_FC_READ_COILS: {
            if (request_len < MODBUS_MBAP_HEADER_SIZE + 5) {
                modbus_build_exception(function_code, MODBUS_EX_ILLEGAL_DATA_VALUE, resp_pdu);
                resp_pdu_len = 2;
                break;
            }
            uint16_t start_addr = (pdu[1] << 8) | pdu[2];
            uint16_t quantity = (pdu[3] << 8) | pdu[4];
            resp_pdu_len = modbus_read_coils(start_addr, quantity, resp_pdu);
            break;
        }

        case MODBUS_FC_READ_DISCRETE_INPUTS: {
            if (request_len < MODBUS_MBAP_HEADER_SIZE + 5) {
                modbus_build_exception(function_code, MODBUS_EX_ILLEGAL_DATA_VALUE, resp_pdu);
                resp_pdu_len = 2;
                break;
            }
            uint16_t start_addr = (pdu[1] << 8) | pdu[2];
            uint16_t quantity = (pdu[3] << 8) | pdu[4];
            resp_pdu_len = modbus_read_discrete_inputs(start_addr, quantity, resp_pdu);
            break;
        }

        case MODBUS_FC_READ_HOLDING_REGISTERS: {
            if (request_len < MODBUS_MBAP_HEADER_SIZE + 5) {
                modbus_build_exception(function_code, MODBUS_EX_ILLEGAL_DATA_VALUE, resp_pdu);
                resp_pdu_len = 2;
                break;
            }
            uint16_t start_addr = (pdu[1] << 8) | pdu[2];
            uint16_t quantity = (pdu[3] << 8) | pdu[4];
            resp_pdu_len = modbus_read_holding_registers(start_addr, quantity, resp_pdu);
            break;
        }

        case MODBUS_FC_WRITE_SINGLE_COIL: {
            if (request_len < MODBUS_MBAP_HEADER_SIZE + 5) {
                modbus_build_exception(function_code, MODBUS_EX_ILLEGAL_DATA_VALUE, resp_pdu);
                resp_pdu_len = 2;
                break;
            }
            uint16_t addr = (pdu[1] << 8) | pdu[2];
            uint16_t value = (pdu[3] << 8) | pdu[4];
            resp_pdu_len = modbus_write_single_coil(addr, value, resp_pdu);
            break;
        }

        case MODBUS_FC_WRITE_MULTIPLE_COILS: {
            if (request_len < MODBUS_MBAP_HEADER_SIZE + 6) {
                modbus_build_exception(function_code, MODBUS_EX_ILLEGAL_DATA_VALUE, resp_pdu);
                resp_pdu_len = 2;
                break;
            }
            uint16_t start_addr = (pdu[1] << 8) | pdu[2];
            uint16_t quantity = (pdu[3] << 8) | pdu[4];
            uint8_t byte_count = pdu[5];
            if (request_len < MODBUS_MBAP_HEADER_SIZE + 6 + byte_count) {
                modbus_build_exception(function_code, MODBUS_EX_ILLEGAL_DATA_VALUE, resp_pdu);
                resp_pdu_len = 2;
                break;
            }
            resp_pdu_len = modbus_write_multiple_coils(start_addr, quantity, &pdu[6], resp_pdu);
            break;
        }

        default:
            // Unsupported function code
            modbus_build_exception(function_code, MODBUS_EX_ILLEGAL_FUNCTION, resp_pdu);
            resp_pdu_len = 2;
            break;
    }

    if (resp_pdu_len <= 0) {
        return -1;
    }

    // Build MBAP header for response
    response[0] = (transaction_id >> 8) & 0xFF;
    response[1] = transaction_id & 0xFF;
    response[2] = 0x00; // Protocol ID high
    response[3] = 0x00; // Protocol ID low
    response[4] = ((resp_pdu_len + 1) >> 8) & 0xFF; // Length high (includes unit_id)
    response[5] = (resp_pdu_len + 1) & 0xFF; // Length low
    response[6] = unit_id;

    return MODBUS_MBAP_HEADER_SIZE + resp_pdu_len;
}

static int16_t modbus_read_coils(uint16_t start_addr, uint16_t quantity, uint8_t* response)
{
    // Validate address range (0-7 for 8 coils)
    if (start_addr + quantity > MODBUS_NUM_DIGITAL_OUTPUTS || quantity == 0 || quantity > 2000) {
        modbus_build_exception(MODBUS_FC_READ_COILS, MODBUS_EX_ILLEGAL_DATA_ADDRESS, response);
        return 2;
    }

    uint8_t outputs = io_read_digital_outputs();
    uint8_t byte_count = (quantity + 7) / 8;

    response[0] = MODBUS_FC_READ_COILS;
    response[1] = byte_count;

    // Pack coil values into response bytes
    for (uint8_t i = 0; i < byte_count; i++) {
        uint8_t byte_val = 0;
        for (uint8_t bit = 0; bit < 8; bit++) {
            uint16_t coil_idx = start_addr + i * 8 + bit;
            if (coil_idx < start_addr + quantity && coil_idx < MODBUS_NUM_DIGITAL_OUTPUTS) {
                if (outputs & (1 << coil_idx)) {
                    byte_val |= (1 << bit);
                }
            }
        }
        response[2 + i] = byte_val;
    }

    return 2 + byte_count;
}

static int16_t modbus_read_discrete_inputs(uint16_t start_addr, uint16_t quantity, uint8_t* response)
{
    // Validate address range (0-7 for 8 inputs)
    if (start_addr + quantity > MODBUS_NUM_DIGITAL_INPUTS || quantity == 0 || quantity > 2000) {
        modbus_build_exception(MODBUS_FC_READ_DISCRETE_INPUTS, MODBUS_EX_ILLEGAL_DATA_ADDRESS, response);
        return 2;
    }

    uint8_t inputs = io_read_digital_inputs();
    uint8_t byte_count = (quantity + 7) / 8;

    response[0] = MODBUS_FC_READ_DISCRETE_INPUTS;
    response[1] = byte_count;

    // Pack input values into response bytes
    for (uint8_t i = 0; i < byte_count; i++) {
        uint8_t byte_val = 0;
        for (uint8_t bit = 0; bit < 8; bit++) {
            uint16_t input_idx = start_addr + i * 8 + bit;
            if (input_idx < start_addr + quantity && input_idx < MODBUS_NUM_DIGITAL_INPUTS) {
                if (inputs & (1 << input_idx)) {
                    byte_val |= (1 << bit);
                }
            }
        }
        response[2 + i] = byte_val;
    }

    return 2 + byte_count;
}

static int16_t modbus_read_holding_registers(uint16_t start_addr, uint16_t quantity, uint8_t* response)
{
    // Validate address range (0-3 for 4 ADC channels)
    if (start_addr + quantity > MODBUS_NUM_ADC_CHANNELS || quantity == 0 || quantity > 125) {
        modbus_build_exception(MODBUS_FC_READ_HOLDING_REGISTERS, MODBUS_EX_ILLEGAL_DATA_ADDRESS, response);
        return 2;
    }

    uint16_t adc_values[MODBUS_NUM_ADC_CHANNELS];
    io_read_adc(adc_values);

    uint8_t byte_count = quantity * 2;

    response[0] = MODBUS_FC_READ_HOLDING_REGISTERS;
    response[1] = byte_count;

    // Pack register values into response (big-endian)
    for (uint16_t i = 0; i < quantity; i++) {
        uint16_t reg_value = adc_values[start_addr + i];
        response[2 + i * 2] = (reg_value >> 8) & 0xFF;
        response[2 + i * 2 + 1] = reg_value & 0xFF;
    }

    return 2 + byte_count;
}

static int16_t modbus_write_single_coil(uint16_t addr, uint16_t value, uint8_t* response)
{
    // Validate address (0-7 for 8 outputs)
    if (addr >= MODBUS_NUM_DIGITAL_OUTPUTS) {
        modbus_build_exception(MODBUS_FC_WRITE_SINGLE_COIL, MODBUS_EX_ILLEGAL_DATA_ADDRESS, response);
        return 2;
    }

    // Value must be 0x0000 (OFF) or 0xFF00 (ON)
    if (value != 0x0000 && value != 0xFF00) {
        modbus_build_exception(MODBUS_FC_WRITE_SINGLE_COIL, MODBUS_EX_ILLEGAL_DATA_VALUE, response);
        return 2;
    }

    // Write the coil
    io_write_digital_output(addr, value == 0xFF00);

    // Echo the request as response
    response[0] = MODBUS_FC_WRITE_SINGLE_COIL;
    response[1] = (addr >> 8) & 0xFF;
    response[2] = addr & 0xFF;
    response[3] = (value >> 8) & 0xFF;
    response[4] = value & 0xFF;

    return 5;
}

static int16_t modbus_write_multiple_coils(uint16_t start_addr, uint16_t quantity, uint8_t* data, uint8_t* response)
{
    // Validate address range (0-7 for 8 outputs)
    if (start_addr + quantity > MODBUS_NUM_DIGITAL_OUTPUTS || quantity == 0 || quantity > 1968) {
        modbus_build_exception(MODBUS_FC_WRITE_MULTIPLE_COILS, MODBUS_EX_ILLEGAL_DATA_ADDRESS, response);
        return 2;
    }

    // Get current outputs and modify
    uint8_t outputs = io_read_digital_outputs();

    // Unpack coil values from data bytes
    for (uint16_t i = 0; i < quantity; i++) {
        uint8_t byte_idx = i / 8;
        uint8_t bit_idx = i % 8;
        bool value = (data[byte_idx] >> bit_idx) & 0x01;

        uint8_t coil_idx = start_addr + i;
        if (value) {
            outputs |= (1 << coil_idx);
        } else {
            outputs &= ~(1 << coil_idx);
        }
    }

    io_write_digital_outputs(outputs);

    // Build response
    response[0] = MODBUS_FC_WRITE_MULTIPLE_COILS;
    response[1] = (start_addr >> 8) & 0xFF;
    response[2] = start_addr & 0xFF;
    response[3] = (quantity >> 8) & 0xFF;
    response[4] = quantity & 0xFF;

    return 5;
}

static void modbus_build_exception(uint8_t function_code, uint8_t exception_code, uint8_t* response)
{
    response[0] = function_code | 0x80; // Set exception bit
    response[1] = exception_code;
}
