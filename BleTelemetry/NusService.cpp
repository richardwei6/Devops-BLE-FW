// Allow referencing the UUID constants elsewhere
// https://stackoverflow.com/questions/8016780/undefined-reference-to-static-constexpr-char
#include "NusService.h"

constexpr uint8_t NusService::kServiceUuid[];
constexpr uint8_t NusService::kRxCharacteristicUuid[];
constexpr uint8_t NusService::kTxCharacteristicUuid[];
