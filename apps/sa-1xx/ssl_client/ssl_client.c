/****************************************************************************
 * SSL client test
 *
 *  Copyright (C) 2013 Century Systems
 *  Author: Takeyoshi Kikuchi <kikuchi@centurysys.co.jp>
 *
 *  Last Modified: 2013/02/05 16:44:19 kikuchi
 ****************************************************************************/

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <termios.h>
#include <fcntl.h>
#include <poll.h>
#include <sys/socket.h>
#include <sys/ioctl.h>
#include <sys/select.h>
#include <errno.h>

#include <apps/netutils/resolv.h>

#include "ssl.h"
#include "os_port.h"

#include "common.h"

#define STDIN_FILENO 0

static int print_client_options(char *option);
static void display_cipher(SSL *ssl);
static void display_session_id(SSL *ssl);

int ssl_client_main(int argc, char **argv)
{
    int res, i = 2;
    uint16_t port = 4433;
    uint32_t options = SSL_SERVER_VERIFY_LATER|SSL_DISPLAY_CERTS;
    int client_fd;
    char *private_key_file = NULL;
    struct sockaddr_in client_addr;
    //struct hostent *hostent;
    int reconnect = 0;
    uint32_t sin_addr;
    SSL_CTX *ssl_ctx;
    SSL *ssl = NULL;
    int quiet = 0;
    int cert_index = 0, ca_cert_index = 0;
    int cert_size, ca_cert_size;
    char **ca_cert, **cert;
    uint8_t session_id[SSL_SESSION_ID_SIZE];
    fd_set read_set;
    const char *password = NULL;

    FD_ZERO(&read_set);
    sin_addr = inet_addr("127.0.0.1");
    cert_size = ssl_get_config(SSL_MAX_CERT_CFG_OFFSET);
    ca_cert_size = ssl_get_config(SSL_MAX_CA_CERT_CFG_OFFSET);
    ca_cert = (char **)calloc(1, sizeof(char *)*ca_cert_size);
    cert = (char **)calloc(1, sizeof(char *)*cert_size);

    while (i < argc)
    {
        if (strcmp(argv[i], "-connect") == 0)
        {
            char *host, *ptr;

            if (i >= argc-1)
            {
                return print_client_options(argv[i]);
            }

            host = argv[++i];
            if ((ptr = strchr(host, ':')) == NULL)
            {
                return print_client_options(argv[i]);
            }

            *ptr++ = 0;
            port = atoi(ptr);
            //hostent = gethostbyname(host);
            //sin_addr = *((uint32_t **)hostent->h_addr_list)[0];
            if (dns_gethostip(host, (in_addr_t *) &sin_addr) < 0)
            {
                return print_client_options(argv[i]);
            }
        }
        else if (strcmp(argv[i], "-cert") == 0)
        {
            if (i >= argc-1 || cert_index >= cert_size)
            {
                return print_client_options(argv[i]);
            }

            cert[cert_index++] = argv[++i];
        }
        else if (strcmp(argv[i], "-key") == 0)
        {
            if (i >= argc-1)
            {
                return print_client_options(argv[i]);
            }

            private_key_file = argv[++i];
            options |= SSL_NO_DEFAULT_KEY;
        }
        else if (strcmp(argv[i], "-CAfile") == 0)
        {
            if (i >= argc-1 || ca_cert_index >= ca_cert_size)
            {
                return print_client_options(argv[i]);
            }

            ca_cert[ca_cert_index++] = argv[++i];
        }
        else if (strcmp(argv[i], "-verify") == 0)
        {
            options &= ~SSL_SERVER_VERIFY_LATER;
        }
        else if (strcmp(argv[i], "-reconnect") == 0)
        {
            reconnect = 4;
        }
        else if (strcmp(argv[i], "-quiet") == 0)
        {
            quiet = 1;
            options &= ~SSL_DISPLAY_CERTS;
        }
        else if (strcmp(argv[i], "-pass") == 0)
        {
            if (i >= argc-1)
            {
                return print_client_options(argv[i]);
            }

            password = argv[++i];
        }
#ifdef CONFIG_SSL_FULL_MODE
        else if (strcmp(argv[i], "-debug") == 0)
        {
            options |= SSL_DISPLAY_BYTES;
        }
        else if (strcmp(argv[i], "-state") == 0)
        {
            options |= SSL_DISPLAY_STATES;
        }
        else if (strcmp(argv[i], "-show-rsa") == 0)
        {
            options |= SSL_DISPLAY_RSA;
        }
#endif
        else    /* don't know what this is */
        {
            return print_client_options(argv[i]);
        }

        i++;
    }

    if ((ssl_ctx = ssl_ctx_new(options, SSL_DEFAULT_CLNT_SESS)) == NULL)
    {
        fprintf(stderr, "Error: Client context is invalid\n");
        exit(1);
    }

    if (private_key_file)
    {
        int obj_type = SSL_OBJ_RSA_KEY;
        
        /* auto-detect the key type from the file extension */
        if (strstr(private_key_file, ".p8"))
            obj_type = SSL_OBJ_PKCS8;
        else if (strstr(private_key_file, ".p12"))
            obj_type = SSL_OBJ_PKCS12;

        if (ssl_obj_load(ssl_ctx, obj_type, private_key_file, password))
        {
            fprintf(stderr, "Error: Private key '%s' is undefined.\n", 
                                                        private_key_file);
            exit(1);
        }
    }

    for (i = 0; i < cert_index; i++)
    {
        if (ssl_obj_load(ssl_ctx, SSL_OBJ_X509_CERT, cert[i], NULL))
        {
            printf("Certificate '%s' is undefined.\n", cert[i]);
            exit(1);
        }
    }

    for (i = 0; i < ca_cert_index; i++)
    {
        if (ssl_obj_load(ssl_ctx, SSL_OBJ_X509_CACERT, ca_cert[i], NULL))
        {
            printf("Certificate '%s' is undefined.\n", ca_cert[i]);
            exit(1);
        }
    }

    free(cert);
    free(ca_cert);

    /*************************************************************************
     * This is where the interesting stuff happens. Up until now we've
     * just been setting up sockets etc. Now we do the SSL handshake.
     *************************************************************************/
    client_fd = socket(AF_INET, SOCK_STREAM, IPPROTO_TCP);
    memset(&client_addr, 0, sizeof(client_addr));
    client_addr.sin_family = AF_INET;
    client_addr.sin_port = htons(port);
    client_addr.sin_addr.s_addr = sin_addr;

    if (connect(client_fd, (struct sockaddr *)&client_addr, 
                sizeof(client_addr)) < 0)
    {
        perror("connect");
        exit(1);
    }

    if (!quiet)
    {
        printf("CONNECTED\n");
        TTY_FLUSH();
    }

    /* Try session resumption? */
    if (reconnect)
    {
        while (reconnect--)
        {
            ssl = ssl_client_new(ssl_ctx, client_fd, session_id,
                    sizeof(session_id));
            if ((res = ssl_handshake_status(ssl)) != SSL_OK)
            {
                if (!quiet)
                {
                    ssl_display_error(res);
                }

                ssl_free(ssl);
                exit(1);
            }

            display_session_id(ssl);
            memcpy(session_id, ssl_get_session_id(ssl), SSL_SESSION_ID_SIZE);

            if (reconnect)
            {
                ssl_free(ssl);
                SOCKET_CLOSE(client_fd);

                client_fd = socket(AF_INET, SOCK_STREAM, IPPROTO_TCP);
                connect(client_fd, (struct sockaddr *)&client_addr, 
                        sizeof(client_addr));
            }
        }
    }
    else
    {
        ssl = ssl_client_new(ssl_ctx, client_fd, NULL, 0);
    }

    /* check the return status */
    if ((res = ssl_handshake_status(ssl)) != SSL_OK)
    {
        if (!quiet)
        {
            ssl_display_error(res);
        }

        exit(1);
    }

    if (!quiet)
    {
        const char *common_name = ssl_get_cert_dn(ssl,
                SSL_X509_CERT_COMMON_NAME);
        if (common_name)
        {
            printf("Common Name:\t\t\t%s\n", common_name);
        }

        display_session_id(ssl);
        display_cipher(ssl);
    }

    for (;;)
    {
        uint8_t buf[1024];

        /* allow parallel reading of server and standard input */
        FD_SET(client_fd, &read_set);
#ifndef WIN32
        /* win32 doesn't like mixing up stdin and sockets */
        FD_SET(STDIN_FILENO, &read_set);

        if ((res = select(client_fd+1, &read_set, NULL, NULL, NULL)) > 0)
        {
            /* read standard input? */
            if (FD_ISSET(STDIN_FILENO, &read_set))
#endif
            {
                if (fgets((char *)buf, sizeof(buf), stdin) == NULL)
                {
                    /* bomb out of here */
                    ssl_free(ssl);
                    break;
                }
                else
                {
                    /* small hack to check renegotiation */
                    if (buf[0] == 'R' && (buf[1] == '\n' || buf[1] == '\r'))
                    {
                        res = ssl_renegotiate(ssl);
                    }
                    else
                    {
                        res = ssl_write(ssl, buf, strlen((char *)buf)+1);
                    }
                }
            }
#ifndef WIN32
            else    /* a socket read */
            {
                uint8_t *read_buf;

                res = ssl_read(ssl, &read_buf);

                if (res > 0)    /* display our interesting output */
                {
                    printf("%s", read_buf);
                    TTY_FLUSH();
                }
            }
        }
#endif

        if (res < 0)
        {
            if (!quiet)
            {
                ssl_display_error(res);
            }

            break;      /* get outta here */
        }
    }

    ssl_ctx_free(ssl_ctx);
    SOCKET_CLOSE(client_fd);

    return 0;
}

/**
 * We've had some sort of command-line error. Print out the client options.
 */
static int print_client_options(char *option)
{
    int cert_size = ssl_get_config(SSL_MAX_CERT_CFG_OFFSET);
    int ca_cert_size = ssl_get_config(SSL_MAX_CA_CERT_CFG_OFFSET);

    printf("unknown option %s\n", option);
    printf("usage: s_client [args ...]\n");
    printf(" -connect host:port - who to connect to (default "
            "is localhost:4433)\n");
    printf(" -verify\t- turn on peer certificate verification\n");
    printf(" -cert arg\t- certificate file to use\n");
    printf("\t\t  Can repeat up to %d times\n", cert_size);
    printf(" -key arg\t- Private key file to use\n");
    printf(" -CAfile arg\t- Certificate authority\n");
    printf("\t\t  Can repeat up to %d times\n", ca_cert_size);
    printf(" -quiet\t\t- No client output\n");
    printf(" -reconnect\t- Drop and re-make the connection "
            "with the same Session-ID\n");
    printf(" -pass\t\t- private key file pass phrase source\n");
#ifdef CONFIG_SSL_FULL_MODE
    printf(" -debug\t\t- Print more output\n");
    printf(" -state\t\t- Show state messages\n");
    printf(" -show-rsa\t- Show RSA state\n");
#endif

    return -1;
}

/**
 * Display what cipher we are using 
 */
static void display_cipher(SSL *ssl)
{
    printf("CIPHER is ");
    switch (ssl_get_cipher_id(ssl))
    {
        case SSL_AES128_SHA:
            printf("AES128-SHA");
            break;

        case SSL_AES256_SHA:
            printf("AES256-SHA");
            break;

        case SSL_RC4_128_SHA:
            printf("RC4-SHA");
            break;

        case SSL_RC4_128_MD5:
            printf("RC4-MD5");
            break;

        default:
            printf("Unknown - %d", ssl_get_cipher_id(ssl));
            break;
    }

    printf("\n");
    TTY_FLUSH();
}

/**
 * Display what session id we have.
 */
static void display_session_id(SSL *ssl)
{
    int i;
    const uint8_t *session_id = ssl_get_session_id(ssl);
    int sess_id_size = ssl_get_session_id_size(ssl);

    if (sess_id_size > 0)
    {
        printf("-----BEGIN SSL SESSION PARAMETERS-----\n");
        for (i = 0; i < sess_id_size; i++)
        {
            printf("%02x", session_id[i]);
        }

        printf("\n-----END SSL SESSION PARAMETERS-----\n");
        TTY_FLUSH();
    }
}
