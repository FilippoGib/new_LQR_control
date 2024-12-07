#ifndef SERIAL_GPS_INTERFACE
#define SERIAL_GPS_INTERFACE

/**
 * This class provides low level interface to retrieve GPS data. 
 * Documentation: https://gpsd.gitlab.io/gpsd/libgps.html
*/

#include <gps.h>
#include <math.h>        // for isfinite()
#include <string>

using namespace std;

/* Read by default 3D data. */
#define MODE_STR_NUM 4
static const char *mode_str[MODE_STR_NUM] = {
    "n/a",
    "None",
    "2D",
    "3D"
};


class SerialGpsInterface
{
    public:
    static bool check_new_data(const gps_data_t old_data, const gps_data_t new_data)
    {
        return ((old_data.fix.latitude != new_data.fix.latitude) || (old_data.fix.longitude != new_data.fix.longitude) || (old_data.fix.altitude != new_data.fix.altitude));
    }

    static int open_connection(string address, string port, gps_data_t &gps_data)
    {
        if (0 != gps_open(address.c_str(), port.c_str(), &gps_data)) {
		perror("Can't connect to GPS");
            return -1;
        }

        (void)gps_stream(&gps_data, WATCH_ENABLE | WATCH_JSON, NULL);

        return 0;
    }

    static int read_data(gps_data_t &gps_data)
    {
        /* Wait for 500 ms before getting new data */
        if (gps_waiting(&gps_data, 500000))
        {
            if (-1 == gps_read(&gps_data, NULL, 0)) {
		perror("Can't read data.");
                return -1;
            }
            
            if (MODE_SET != (MODE_SET & gps_data.set)) {
                // did not even get mode, nothing to see here
                return -1;
            }
            if (0 > gps_data.fix.mode ||
                MODE_STR_NUM <= gps_data.fix.mode) {
                gps_data.fix.mode = 0;
            }
            if (isfinite(gps_data.fix.latitude) &&
                isfinite(gps_data.fix.longitude)) {
                return 0;
                // printf("Lat %.6f Lon %.6f\n",
                //         gps_data.fix.latitude, gps_data.fix.longitude);
            } else {
                return -1;
            }
        }
        return -1;
    }

    static int close_connection(gps_data_t &gps_data)
    {
        (void)gps_stream(&gps_data, WATCH_DISABLE, NULL);
        return gps_close(&gps_data);
    }
};

#endif /* SERIAL_GPS_INTERFACE */
