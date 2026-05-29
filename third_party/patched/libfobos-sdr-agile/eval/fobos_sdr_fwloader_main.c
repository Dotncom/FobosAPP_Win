//==============================================================================
//  Fobos SDR (agile) special API library test application
//  firmware read-write example
//  V.T.
//  LGPL-2.1+
//  2025.01.21
//==============================================================================
#include <stdio.h>
#ifdef _WIN32
#include <Windows.h>
#endif
#include <string.h>
#include <fobos_sdr.h>
//==============================================================================

//==============================================================================
int main(int argc, char** argv)
{
    printf("fobos sdr (agile) special firmware loader tool\n");
    printf("machine: x%d\n", (int)sizeof(void*)*8);
    for(int i = 0; i < argc; i++)
    {
        printf("arg[%d]=%s\n", i, argv[i]);    
    }
    if (argc > 2)
    {
        const char * file_name = argv[2];
        struct fobos_sdr_dev_t * dev = NULL;
        int count = fobos_sdr_get_device_count();
        int result = 0;
        int index = 0;
        if (count > 0)
        {
            printf("found devices: %d\n", count);
            result = fobos_sdr_open(&dev, index);
            if (result == FOBOS_ERR_OK)
            {
                if (strcmp(argv[1], "-w") == 0)
                {
                    result = fobos_sdr_write_firmware(dev, file_name, 1);
                    fobos_sdr_reset(dev);  // reboot in a new firmware
                }
                if (strcmp(argv[1], "-r") == 0)
                {
                    result = fobos_sdr_read_firmware(dev, file_name, 1);
                    fobos_sdr_close(dev);  // close the device in regular mode
                }
            }
            else
            {
                printf("could not open device! err (%i)\n", result);
            }
        }
    }
    else
    {
        printf("usage: \n    %s -r firmware/saved/to/file.bin\n    %s -w firmware/loaded/from/file.bin\n", argv[0], argv[0]);
    }
    return 0;
}
//==============================================================================
