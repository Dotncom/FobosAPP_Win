//==============================================================================
//  Fobos SDR (agile) API library test application
//  V.T.
//  LGPL-2.1+
//  2024.12.07
//==============================================================================
#include <stdio.h>
#include <string.h>
#include <fobos_sdr.h>
//==============================================================================
void get_devinfo()
{
    struct fobos_sdr_dev_t * dev = NULL;
    int result = 0;
    char lib_version[64];
    char drv_version[64];
    char serials[256] = {0};

    int index = 0;

    char hw_revision[64];
    char fw_version[64];
    char manufacturer[64];
    char product[64];
    char serial[64];
    
    fobos_sdr_get_api_info(lib_version, drv_version);

    printf("API Info lib: %s drv: %s\n", lib_version, drv_version);

    int count = fobos_sdr_list_devices(serials);
    char* pserial = strtok(serials, " ");

    printf("found devices: %d\n", count);

    if (count > 0)
    {
        for (index = 0; index < count; index++)
        {
            printf("  dev# %i  %s\n", index, pserial);
            pserial = strtok(0, " ");

            result = fobos_sdr_open(&dev, index);

            if (result == 0)
            {
                result = fobos_sdr_get_board_info(dev, hw_revision, fw_version, manufacturer, product, serial);
                if (result != 0)
                {
                    printf("fobos_sdr_get_board_info - error!\n");
                }
                else
                {
                    printf("    hw_revision:  %s\n", hw_revision);
                    printf("    fw_version:   %s\n", fw_version);
                    printf("    manufacturer: %s\n", manufacturer);
                    printf("    product:      %s\n", product);
                    printf("    serial:       %s\n", serial);
                }
                fobos_sdr_close(dev);
            }
            else
            {
                printf("    could not open device\n");
            }
        }
    }
}
//==============================================================================
int main(int argc, char** argv)
{
    printf("Fobos SDR (agile) get device info test app\n");
    get_devinfo();
    return 0;
}
//==============================================================================
