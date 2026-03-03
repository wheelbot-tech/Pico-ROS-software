/*******************************************************************************
 * @file    common.c
 * @brief   Common utilities for picoros examples
 * @date    2025-May-27
 *
 * @details This file provides common utility functions used across
 *          the example programs,
 *
 * @copyright Copyright (c) 2025 Ubiquity Robotics
 *******************************************************************************/

#include "picoros.h"
#include <stdio.h>
#include <unistd.h>
#include <signal.h>

volatile sig_atomic_t picoros_keep_running = 1;

void handle_sigint(int sig) {
    (void)sig;
    printf("\nGot SIGINT.\n");
    if (picoros_keep_running == 0){
        printf("Forcing stop.\n");
        exit(-1);
    }
    picoros_keep_running = 0;
}

void sys_setup_sigint_handler(void) {
    signal(SIGINT, handle_sigint);
}

int sys_parse_args(int argc, char **argv, picoros_interface_t* ifx) {
    int opt;
    while ((opt = getopt(argc, argv, "a:m:h")) != -1) {
        switch (opt) {
            case 'a':
                // CONFIG_CONNECT_KEY
                ifx->locator = optarg;
                break;
            case 'm':
                // Z_CONFIG_MODE_KEY
                ifx->mode = optarg;
                break;
            case 'h':
                fprintf(stderr,
                    "-m 'mode' ['client', 'peer'] \n"
                    "-a 'address' to connect or listen on (ex: 'tcp/192.168.1.16:7447', 'udp/224.0.0.225:7447#iface=en0')\n"
                );
            case '?':
                if (optopt == 'a' || optopt == 'm') {
                    fprintf(stderr, "Option -%c requires an argument.\n", optopt);
                }
                return 1;
                break;
            default:
                return -1;
                break;

        }
    }
    return 0;
}
