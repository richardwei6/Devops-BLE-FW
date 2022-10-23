// Allow referencing the UUID
// https://stackoverflow.com/questions/8016780/undefined-reference-to-static-constexpr-char
#include "MultimeterService.h"

constexpr uint8_t MultimeterService::kUuidService[];
constexpr uint8_t MultimeterService::kUuidReading[];
constexpr uint8_t MultimeterService::kUuidMode[];
constexpr uint8_t MultimeterService::kUuidAdc[];
constexpr uint8_t MultimeterService::kUuidResistance[];
