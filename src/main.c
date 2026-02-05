#include <stdio.h>
#include <string.h>

#include "pico/stdlib.h"
#include "pico/unique_id.h"
#include "hardware/timer.h"

#include "w6100_spi.h"
#include "wizchip_conf.h"
#include "w6100.h"
#include "socket.h"
#include "dhcp.h"

#include "io_handler.h"
#include "modbus_server.h"

// Network configuration
#define DHCP_SOCKET      7  // Use socket 7 for DHCP (socket 0 is for Modbus)
#define DHCP_BUFFER_SIZE 1024

// Timer interval for DHCP (1 second)
#define DHCP_TIMER_INTERVAL_US 1000000

// DHCP buffer
static uint8_t g_dhcp_buffer[DHCP_BUFFER_SIZE];

// Network info storage
static uint8_t g_mac_addr[6];
static uint8_t g_ip_addr[4] = {0, 0, 0, 0};
static uint8_t g_subnet[4] = {255, 255, 255, 0};
static uint8_t g_gateway[4] = {0, 0, 0, 0};

// DHCP state
static volatile bool g_dhcp_got_ip = false;
static struct repeating_timer g_dhcp_timer;

// DHCP callbacks
static void dhcp_ip_assign(void);
static void dhcp_ip_update(void);
static void dhcp_ip_conflict(void);

// Timer callback for DHCP
static bool dhcp_timer_callback(struct repeating_timer *t);

// Initialize network
static bool network_init(void);
static void generate_mac_address(uint8_t* mac);
static void print_network_info(void);

int main(void)
{
    // Initialize stdio for USB serial output
    stdio_init_all();
    sleep_ms(1000);

    printf("\n");
    printf("========================================\n");
    printf("Pico Modbus TCP Server\n");
    printf("W6100-EVB-Pico2\n");
    printf("========================================\n");

    // Initialize I/O (ADC and GPIO)
    io_handler_init();

    // Initialize W6100 SPI interface
    w6100_spi_init();

    // Initialize network (W6100 chip and DHCP)
    if (!network_init()) {
        printf("ERROR: Failed to initialize network!\n");
        while (1) {
            sleep_ms(1000);
        }
    }

    // Initialize Modbus server
    modbus_server_init();

    printf("Waiting for DHCP...\n");

    // Main loop
    while (1) {
        // Run DHCP state machine
        uint8_t dhcp_status = DHCP_run();

        if (dhcp_status == DHCP_IP_LEASED || g_dhcp_got_ip) {
            // Run Modbus server
            modbus_server_run();
        }

        // Small delay to prevent busy-waiting
        sleep_us(100);
    }

    return 0;
}

static bool network_init(void)
{
    uint8_t tmp;
    // TX buffer sizes (8 sockets) + RX buffer sizes (8 sockets) = 16 bytes
    // 2KB per socket for both TX and RX
    uint8_t mem_size[16] = {2, 2, 2, 2, 2, 2, 2, 2,   // TX: 2KB each
                           2, 2, 2, 2, 2, 2, 2, 2};  // RX: 2KB each

    // Generate MAC address from Pico unique ID
    generate_mac_address(g_mac_addr);

    printf("MAC Address: %02X:%02X:%02X:%02X:%02X:%02X\n",
           g_mac_addr[0], g_mac_addr[1], g_mac_addr[2],
           g_mac_addr[3], g_mac_addr[4], g_mac_addr[5]);

    // Check if W6100 is responding
    if (ctlwizchip(CW_GET_PHYLINK, &tmp) != 0) {
        printf("ERROR: W6100 not responding!\n");
        return false;
    }

    // Initialize W6100 memory
    if (ctlwizchip(CW_INIT_WIZCHIP, mem_size) != 0) {
        printf("ERROR: Failed to initialize W6100 memory!\n");
        return false;
    }

    // Unlock network registers before setting MAC/IP/etc
    NETUNLOCK();

    // Set MAC address
    setSHAR(g_mac_addr);

    // Set network mode to IPv4
    setNET4MR(0x00);

    // Register DHCP callbacks
    reg_dhcp_cbfunc(dhcp_ip_assign, dhcp_ip_update, dhcp_ip_conflict);

    // Initialize DHCP
    DHCP_init(DHCP_SOCKET, g_dhcp_buffer);

    // Start DHCP timer (1 second interval)
    add_repeating_timer_us(DHCP_TIMER_INTERVAL_US, dhcp_timer_callback, NULL, &g_dhcp_timer);

    return true;
}

static void generate_mac_address(uint8_t* mac)
{
    pico_unique_board_id_t board_id;
    pico_get_unique_board_id(&board_id);

    // Use WIZnet OUI prefix and unique board ID
    mac[0] = 0x00;  // WIZnet OUI
    mac[1] = 0x08;
    mac[2] = 0xDC;
    mac[3] = board_id.id[5];
    mac[4] = board_id.id[6];
    mac[5] = board_id.id[7];
}

static void dhcp_ip_assign(void)
{
    getIPfromDHCP(g_ip_addr);
    getGWfromDHCP(g_gateway);
    getSNfromDHCP(g_subnet);

    // Apply network configuration
    setSIPR(g_ip_addr);
    setGAR(g_gateway);
    setSUBR(g_subnet);

    g_dhcp_got_ip = true;

    printf("\n");
    printf("========================================\n");
    printf("DHCP IP Assigned!\n");
    print_network_info();
    printf("Modbus TCP Server ready on port %d\n", MODBUS_TCP_PORT);
    printf("========================================\n");
}

static void dhcp_ip_update(void)
{
    getIPfromDHCP(g_ip_addr);
    getGWfromDHCP(g_gateway);
    getSNfromDHCP(g_subnet);

    // Apply network configuration
    setSIPR(g_ip_addr);
    setGAR(g_gateway);
    setSUBR(g_subnet);

    printf("\nDHCP IP Updated!\n");
    print_network_info();
}

static void dhcp_ip_conflict(void)
{
    printf("ERROR: DHCP IP conflict detected!\n");
    g_dhcp_got_ip = false;
}

static bool dhcp_timer_callback(struct repeating_timer *t)
{
    (void)t;
    DHCP_time_handler();
    return true;
}

static void print_network_info(void)
{
    printf("IP Address: %d.%d.%d.%d\n",
           g_ip_addr[0], g_ip_addr[1], g_ip_addr[2], g_ip_addr[3]);
    printf("Subnet:     %d.%d.%d.%d\n",
           g_subnet[0], g_subnet[1], g_subnet[2], g_subnet[3]);
    printf("Gateway:    %d.%d.%d.%d\n",
           g_gateway[0], g_gateway[1], g_gateway[2], g_gateway[3]);
}
