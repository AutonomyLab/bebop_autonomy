// Define the structs to extract the Metadata from the Bebop Video Stream


// Basic metadata struct
typedef struct
{
    uint16_t specific;             /**< Identifier = 0x5031 */
    uint16_t length;               /**< Size in 32 bits words = 6 */
    int16_t  droneYaw;             /**< Drone yaw/psi (rad), Q4.12 */
    int16_t  dronePitch;           /**< Drone pitch/theta (rad), Q4.12 */
    int16_t  droneRoll;            /**< Drone roll/phi (rad), Q4.12 */
    int16_t  cameraPan;            /**< Camera pan (rad), Q4.12 */
    int16_t  cameraTilt;           /**< Camera tilt (rad), Q4.12 */
    int16_t  frameW;               /**< Frame view quaternion W, Q4.12 */
    int16_t  frameX;               /**< Frame view quaternion X, Q4.12 */
    int16_t  frameY;               /**< Frame view quaternion Y, Q4.12 */
    int16_t  frameZ;               /**< Frame view quaternion Z, Q4.12 */
    int16_t  exposureTime;         /**< Frame exposure time (ms), Q8.8 */
    int16_t  gain;                 /**< Frame ISO gain */
    int8_t   wifiRssi;             /**< Wifi RSSI (dBm) */
    uint8_t  batteryPercentage;    /**< Battery charge percentage */
} StreamingMetadataV1Basic_t;

// Extended metadata structure
typedef struct
{
    uint16_t specific;             /**< Identifier = 0x5031 */
    uint16_t length;               /**< Size in 32 bits words = 13 */
    int16_t  droneYaw;             /**< Drone yaw/psi (rad), Q4.12 */
    int16_t  dronePitch;           /**< Drone pitch/theta (rad), Q4.12 */
    int16_t  droneRoll;            /**< Drone roll/phi (rad), Q4.12 */
    int16_t  cameraPan;            /**< Camera pan (rad), Q4.12 */
    int16_t  cameraTilt;           /**< Camera tilt (rad), Q4.12 */
    int16_t  frameW;               /**< Frame view quaternion W, Q4.12 */
    int16_t  frameX;               /**< Frame view quaternion X, Q4.12 */
    int16_t  frameY;               /**< Frame view quaternion Y, Q4.12 */
    int16_t  frameZ;               /**< Frame view quaternion Z, Q4.12 */
    int16_t  exposureTime;         /**< Frame exposure time (ms), Q8.8 */
    int16_t  gain;                 /**< Frame ISO gain */
    int8_t   wifiRssi;             /**< Wifi RSSI (dBm) */
    uint8_t  batteryPercentage;    /**< Battery charge percentage */
    int32_t  gpsLatitude;          /**< GPS latitude (deg), Q12.20 */
    int32_t  gpsLongitude;         /**< GPS longitude (deg), Q12.20 */
    int32_t  gpsAltitudeAndSv;     /**< Bits 31..8 = GPS altitude (m) Q16.8, bits 7..0 = SV count */
    int32_t  altitude;             /**< Altitude relative to take-off (m), Q16.16 */
    uint32_t distanceFromHome;     /**< Distance from home (m), Q16.16 */
    int16_t  xSpeed;               /**< X speed (m/s), Q8.8 */
    int16_t  ySpeed;               /**< Y speed (m/s), Q8.8 */
    int16_t  zSpeed;               /**< Z speed (m/s), Q8.8 */
    uint8_t  state;                /**< Bit 7 = binning, bits 6..0 = flyingState */
    uint8_t  mode;                 /**< Bit 7 = animation, bits 6..0 = pilotingMode */
} StreamingMetadataV1Extended_t;
