// Define the structs to extract the Metadata from the Bebop Video Stream
typedef struct
{
uint16_t id; /**< Identifier = 0x5032 */
uint16_t length; /**< Structure size in 32 bits words excluding the id and length
fields and including extensions */
int32_t groundDistance; /**< Best ground distance estimation (m), Q16.16 */
int32_t latitude; /**< Absolute latitude (deg), Q10.22 */
int32_t longitude; /**< Absolute longitude (deg), Q10.22 */
int32_t altitudeAndSv; /**< Bits 31..8 = altitude (m) Q16.8, bits 7..0 = GPS SV count */
int16_t northSpeed; /**< North speed (m/s), Q8.8 */
int16_t eastSpeed; /**< East speed (m/s), Q8.8 */
int16_t downSpeed; /**< Down speed (m/s), Q8.8 */
int16_t airSpeed; /**< Speed relative to air (m/s), negative means no data, Q8.8 */
int16_t droneW; /**< Drone quaternion W, Q2.14 */
int16_t droneX; /**< Drone quaternion X, Q2.14 */
int16_t droneY; /**< Drone quaternion Y, Q2.14 */
int16_t droneZ; /**< Drone quaternion Z, Q2.14 */
int16_t frameW; /**< Frame view quaternion W, Q2.14 */
int16_t frameX; /**< Frame view quaternion X, Q2.14 */
int16_t frameY; /**< Frame view quaternion Y, Q2.14 */
int16_t frameZ; /**< Frame view quaternion Z, Q2.14 */
int16_t cameraPan; /**< Camera pan (rad), Q4.12 */
int16_t cameraTilt; /**< Camera tilt (rad), Q4.12 */
uint16_t exposureTime; /**< Frame exposure time (ms), Q8.8 */
uint16_t gain; /**< Frame ISO gain */
uint8_t state; /**< Bit 7 = binning, bits 6..0 = flyingState */
uint8_t mode; /**< Bit 7 = animation, bits 6..0 = pilotingMode */
int8_t wifiRssi; /**< Wifi RSSI (dBm) */
uint8_t batteryPercentage; /**< Battery charge percentage */
} MetadataV2Base_t;
