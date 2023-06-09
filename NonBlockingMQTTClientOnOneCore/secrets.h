//
// Created by timf3 on 08/12/2022.
//

#ifndef FOV_FOOTBALLFIRMWARE_SECRETS_H
#define FOV_FOOTBALLFIRMWARE_SECRETS_H

#endif //FOV_FOOTBALLFIRMWARE_SECRETS_H

#include <pgmspace.h>

#define SECRET
#define THINGNAME "belfast_baord_3"                                       //change this

bool omar_dev_board = true;

//const char WIFI_SSID[] = "DownTown-Guest";                                        //change this
//const char WIFI_PASSWORD[] = "lmdt@2022";
const char WIFI_SSID[] = "Redmi Note 10 Pro";                                        //change this
const char WIFI_PASSWORD[] = "aaaaaaaa";
//change this
//const char WIFI_SSID[] = "HUAWEI nova 5T";
//const char WIFI_PASSWORD[] = "monkeys123";
const char AWS_IOT_ENDPOINT[] = "a13d7wu4wem7v1-ats.iot.eu-west-1.amazonaws.com";        //change this


// Amazon Root CA 1
static const char AWS_CERT_CA[] PROGMEM = R"EOF(
-----BEGIN CERTIFICATE-----
MIIDQTCCAimgAwIBAgITBmyfz5m/jAo54vB4ikPmljZbyjANBgkqhkiG9w0BAQsF
ADA5MQswCQYDVQQGEwJVUzEPMA0GA1UEChMGQW1hem9uMRkwFwYDVQQDExBBbWF6
b24gUm9vdCBDQSAxMB4XDTE1MDUyNjAwMDAwMFoXDTM4MDExNzAwMDAwMFowOTEL
MAkGA1UEBhMCVVMxDzANBgNVBAoTBkFtYXpvbjEZMBcGA1UEAxMQQW1hem9uIFJv
b3QgQ0EgMTCCASIwDQYJKoZIhvcNAQEBBQADggEPADCCAQoCggEBALJ4gHHKeNXj
ca9HgFB0fW7Y14h29Jlo91ghYPl0hAEvrAIthtOgQ3pOsqTQNroBvo3bSMgHFzZM
9O6II8c+6zf1tRn4SWiw3te5djgdYZ6k/oI2peVKVuRF4fn9tBb6dNqcmzU5L/qw
IFAGbHrQgLKm+a/sRxmPUDgH3KKHOVj4utWp+UhnMJbulHheb4mjUcAwhmahRWa6
VOujw5H5SNz/0egwLX0tdHA114gk957EWW67c4cX8jJGKLhD+rcdqsq08p8kDi1L
93FcXmn/6pUCyziKrlA4b9v7LWIbxcceVOF34GfID5yHI9Y/QCB/IIDEgEw+OyQm
jgSubJrIqg0CAwEAAaNCMEAwDwYDVR0TAQH/BAUwAwEB/zAOBgNVHQ8BAf8EBAMC
AYYwHQYDVR0OBBYEFIQYzIU07LwMlJQuCFmcx7IQTgoIMA0GCSqGSIb3DQEBCwUA
A4IBAQCY8jdaQZChGsV2USggNiMOruYou6r4lK5IpDB/G/wkjUu0yKGX9rbxenDI
U5PMCCjjmCXPI6T53iHTfIUJrU6adTrCC2qJeHZERxhlbI1Bjjt/msv0tadQ1wUs
N+gDS63pYaACbvXy8MWy7Vu33PqUXHeeE6V/Uq2V8viTO96LXFvKWlJbYK8U90vv
o/ufQJVtMVT8QtPHRh8jrdkPSHCa2XV4cdFyQzR1bldZwgJcJmApzyMZFo6IQ6XU
5MsI+yMRQ+hDKXJioaldXgjUkK642M4UwtBV8ob2xJNDd2ZhwLnoQdeXeGADbkpy
rqXRfboQnoZsG4q5WTP468SQvvG5
-----END CERTIFICATE-----
)EOF";

// Device Certificate                                               //change this
static const char AWS_CERT_CRT[] PROGMEM = R"KEY(
-----BEGIN CERTIFICATE-----
MIIDWjCCAkKgAwIBAgIVANOe7F2UstBsK3MN1ecVPBcGjsfPMA0GCSqGSIb3DQEB
CwUAME0xSzBJBgNVBAsMQkFtYXpvbiBXZWIgU2VydmljZXMgTz1BbWF6b24uY29t
IEluYy4gTD1TZWF0dGxlIFNUPVdhc2hpbmd0b24gQz1VUzAeFw0yMzAzMjUxMjA5
NTRaFw00OTEyMzEyMzU5NTlaMB4xHDAaBgNVBAMME0FXUyBJb1QgQ2VydGlmaWNh
dGUwggEiMA0GCSqGSIb3DQEBAQUAA4IBDwAwggEKAoIBAQDAH071sgL6dwtseY7h
TxtTjtIckNnGeBBs0M4sJC7M2/iED0uvY36xyvhEoIlWZRTG5CgJ1Ai8QtBTLseR
XW4uy7tFYpH8gleGgSiT6kCstsyjErfNWEAiiBa/DJCtPTpsOjE4nA+U/VyVmlaI
mFiZx2DNHdu7CBSoAby0XTvowp755z6tJEArL51rVLNoHkXr6aR9rcitcKjf0IpZ
LFBbw6cHe4VKGdUGDYl3BBUB7eK6WOwYSLSTwDPUGCRd6zEG8yO41bren7hIw5wo
sJiEicCMiuujw2blxhBfq15j2SMu/hFKIkWVH3Un03kKApCIm1XtLXjolasUFVeN
ZVuZAgMBAAGjYDBeMB8GA1UdIwQYMBaAFH1BbiSEjtYC+HCXm4nwQ8JXRA2nMB0G
A1UdDgQWBBSTowcAi4PUNVurt37cR+ejSgK6dzAMBgNVHRMBAf8EAjAAMA4GA1Ud
DwEB/wQEAwIHgDANBgkqhkiG9w0BAQsFAAOCAQEAIIj7nIRXY1ZJcVlPn5DfJoOT
2P2Dkh8mVLn1eVWzVHC2Zxe1MY3UGMQLLOV8Qr8Y/c0g2SDj/4uQXOpQeAOb9cee
ylmcMU00qxxfF+PU5e1Xp0TSOAe/ZLXNUFvzkJofWFsKEe3XW+F1U2J2Sp/zH8xf
rixDEegXIWPqYZgX3kOV1SBkDEmbB+VWQRK7u9J3CwVY+LxVFjgzBlX76SDBNpJ4
9JQeKGV9y9q0h2H3wlnDr6sW5P1gpL+BZQPORSOO30/gQ1/7yWcqkwrlD2ygrqCx
z8A9XmQQajs3AqHk5QDXQiOs3lnex7xOQ9iv6QcaNoaK/MnTJSk+dklKH7QM7g==
-----END CERTIFICATE-----
)KEY";

// Device Private Key                                               //change this
static const char AWS_CERT_PRIVATE[] PROGMEM = R"KEY(
-----BEGIN RSA PRIVATE KEY-----
MIIEpAIBAAKCAQEAwB9O9bIC+ncLbHmO4U8bU47SHJDZxngQbNDOLCQuzNv4hA9L
r2N+scr4RKCJVmUUxuQoCdQIvELQUy7HkV1uLsu7RWKR/IJXhoEok+pArLbMoxK3
zVhAIogWvwyQrT06bDoxOJwPlP1clZpWiJhYmcdgzR3buwgUqAG8tF076MKe+ec+
rSRAKy+da1SzaB5F6+mkfa3IrXCo39CKWSxQW8OnB3uFShnVBg2JdwQVAe3iuljs
GEi0k8Az1BgkXesxBvMjuNW63p+4SMOcKLCYhInAjIrro8Nm5cYQX6teY9kjLv4R
SiJFlR91J9N5CgKQiJtV7S146JWrFBVXjWVbmQIDAQABAoIBAQCt/+0IUselxglf
nhxMbNPn8nLTElFaEcV2eAoyKjcErLjACgZW+3p0VcpShT69M2EE3QByHkonDzAU
FksCmaa/1R5vsd1b30Sb+gAhLsiSZCCkFMDIxoOXgr9vGlcxv0FZYb7QcW8fVfPz
Pkve/HNKGyvNuZwXITNsEd6xT7uBiqenuXNo7wxHWWCj+JLfmug8s7eQxD9ylKdW
UyHBbuyi32tnDinT+X4Jfv84WMexBJPrxTavj3XEDaDar/g0qLrCVCYBtzu7lsOs
zRZNwGZ0+23RWnGC9uWpkoXDvOULdNBfYI+haiTPpfVqrsRq6Hti0R8x4KgAIIEn
xw9plbd9AoGBAO5Cg+MaF4Ksai+xm7FM9VtOfiPZzdOyKACzHCaJdeADx6Tjl218
Rzsjs4LH6CaktdrCarCVolqnd7448Vd/mWQZogOkNGXozZBX4a6GJFt1Bz5CC5Pd
+4o4ZBjv8Uwx9wPv7seiD2sNx8g5q68Ja8QEWs10ICWALBzjNuGI+2onAoGBAM5t
XPPIsXrYHfz2zYwskXO8jBqCgFuRWduPaAb+FEFvn7eIBRetpn75qL17VMbWCMuJ
fq2hPLlE2pBR0njquENtZIy9SBoYxGwUgUBOzqiG2Po7qNOAyaYzn//atotAY3zk
9Nplzg0ZkXT4YpjJpmx6H3XxrxLbC9UAQ7aB2WQ/AoGBAK4hLLSYiCA3lC+hNXGC
FjuSBTx8XSJfrT69qFA93ElJWeqXbz9UOdbuA68MHQ3JCSOFPWYrJgntJOWARlo1
ZkyzMyZIJwdn+nyIzea4IPA6en201TNPZBwjlHxpOEgAqFBXVYLPXUdXPfCGvYPi
dcE/kYFgYnIl15eRM7XnYB+3AoGAFZnoIjKcL+ixqqkA9WwshFn0p0mEuRBKwOkO
z7yCzh4qLoBzV/j59UuW1s0zH+OB5BqtQOogepqz7GpDhhe51IR8AyZDh0eUNmMm
613TzlreFcFsd0WA1H2Ofq2acJP6VTV4UepZV5GXYRAk8SwY2D0nUaZK1cRKRq3S
aCTKZO0CgYB2E0672RQvh9lgmTfeus16ssTM9+Zk5CIALhRxhkwxq4ik5HXtaHdv
TBBhG5h0cYTLMNrHLkZOuNQPqbUOKpkHZPVFbZw3sY91nk9L1oehWkrusoc3mUGM
RnXbbYz4TQcM21rH3ub/jUAoqqCIOwfXlfGYTfuQXY8zfthgiDEcIw==
-----END RSA PRIVATE KEY-----
)KEY";