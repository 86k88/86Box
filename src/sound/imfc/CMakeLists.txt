# CMakeLists.txt for IMFC device (Z80 + PIU)

set(IMFC_SOURCES
    piu.c         # PIU handshake module
    ppi_v2.c      # your PPI implementation (rename if you used a different filename)
	opm.c
	i8253.c
)

set(IMFC_HEADERS
    piu.h
    ppi_v2.h
	opm.h
	i8253.h
)

# create an object library so it can be linked into 86Box devices
add_library(dev_imfc OBJECT
    ${IMFC_SOURCES}
    ${IMFC_HEADERS}
)
# make sure we use same C standard as the rest of 86Box
set_target_properties(dev_imfc PROPERTIES
    C_STANDARD 99
    C_STANDARD_REQUIRED YES
)

