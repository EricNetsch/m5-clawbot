cmake_minimum_required(VERSION 3.16)

set(EXTRA_COMPONENT_DIRS
    components/esp-openclaw-node
    components/esp-openclaw-node-provisioning
    third_party/microlink/components)
set(COMPONENTS main)
set(PROJECT_VER "1.0.0")

include($ENV{IDF_PATH}/tools/cmake/project.cmake)

# Enable lwIP local DNS hostlist (dynamic) so tailnet MagicDNS names
# (e.g. <your-host>.<your-tailnet>.ts.net) can be resolved locally against the
# MicroLink peer table. This lets the gateway wss:// URI use the real hostname
# so TLS SNI + cert common-name both match the LetsEncrypt Serve cert while the
# packets route over the WireGuard tunnel. Not exposed via IDF Kconfig, so inject
# the raw lwIP options globally (both lwip and the app must see the same value
# for the dns.h declarations/#if guards to line up). Must be set before project().
idf_build_set_property(COMPILE_DEFINITIONS "DNS_LOCAL_HOSTLIST=1" APPEND)
idf_build_set_property(COMPILE_DEFINITIONS "DNS_LOCAL_HOSTLIST_IS_DYNAMIC=1" APPEND)

project(esp_openclaw_node_esp32)
