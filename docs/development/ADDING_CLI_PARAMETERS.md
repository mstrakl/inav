# Adding CLI Configuration Parameters in INAV

This guide explains how to add new CLI configuration parameters to INAV that can be configured from the INAV Configurator.

## Overview

INAV uses a parameter group system to manage configuration. Parameters are defined in:
1. Header files (`.h`) - Structure definitions
2. Source files (`.c`) - Registration and defaults
3. `settings.yaml` - CLI parameter definitions
4. `parameter_group_ids.h` - Parameter group identifiers

The build system automatically generates CLI code from `settings.yaml`.

## Step-by-Step Guide

### 1. Define the Configuration Structure

In your header file (e.g., `src/main/sensors/rangefinder.h`):

```c
typedef struct rangefinderConfig_s {
    uint8_t rangefinder_hardware;
    uint8_t use_median_filtering;
    // Add your new fields here
} rangefinderConfig_t;

PG_DECLARE(rangefinderConfig_t, rangefinderConfig);
```

### 2. Register the Parameter Group

In the corresponding source file (e.g., `src/main/sensors/rangefinder.c`):

```c
PG_REGISTER_WITH_RESET_TEMPLATE(rangefinderConfig_t, rangefinderConfig, PG_RANGEFINDER_CONFIG, 3);

PG_RESET_TEMPLATE(rangefinderConfig_t, rangefinderConfig,
    .rangefinder_hardware = SETTING_RANGEFINDER_HARDWARE_DEFAULT,
    .use_median_filtering = SETTING_RANGEFINDER_MEDIAN_FILTER_DEFAULT,
    // Add your new field defaults here
);
```

**Note:** The last parameter in `PG_REGISTER_WITH_RESET_TEMPLATE` is the version number. Increment this when making changes to the structure.

### 3. Define Parameter Group ID (if creating new group)

In `src/main/config/parameter_group_ids.h`:

- For new INAV-specific groups, use IDs in the range 1000-4094
- Add your ID between `PG_INAV_START` (1000) and `PG_INAV_END`
- Update `PG_INAV_END` to point to your new ID

```c
#define PG_INAV_START 1000
// ... existing IDs ...
#define PG_YOUR_NEW_CONFIG 1045  // Next available ID
#define PG_INAV_END PG_YOUR_NEW_CONFIG
```

### 4. Add Settings to settings.yaml

In `src/main/fc/settings.yaml`:

#### 4.1 Add Table Definitions (if needed)

At the top of the file (lines 1-150), add value tables for enum-based parameters:

```yaml
tables:
  - name: your_table_name
    values: ["OPTION1", "OPTION2", "OPTION3"]
    enum: yourEnum_e  # Optional: links to C enum type
```

#### 4.2 Add Parameter Group

```yaml
  - name: PG_YOUR_CONFIG
    type: yourConfig_t
    headers: ["path/to/your_header.h"]
    condition: USE_YOUR_FEATURE  # Optional: only include if feature enabled
    members:
      - name: your_parameter_name
        description: "Description shown in CLI help"
        default_value: DEFAULT_VALUE
        table: your_table_name  # For enum/table-based values
        min: 0      # For numeric values
        max: 100    # For numeric values
        type: bool  # Type: bool, uint8_t, uint16_t, uint32_t, int8_t, etc.
        field: actual_struct_field_name  # If different from 'name'
```

### 5. Build the Firmware

```bash
bash build.sh SITL
```

The build system will automatically generate the CLI code from `settings.yaml`.

## Complete Example: Adding a Parameter to Rangefinder

### Example 1: Adding a Boolean Parameter

**Header file** (`src/main/sensors/rangefinder.h`):
```c
typedef struct rangefinderConfig_s {
    uint8_t rangefinder_hardware;
    uint8_t use_median_filtering;
    uint8_t use_advanced_filtering;  // NEW
} rangefinderConfig_t;
```

**Source file** (`src/main/sensors/rangefinder.c`):
```c
PG_REGISTER_WITH_RESET_TEMPLATE(rangefinderConfig_t, rangefinderConfig, PG_RANGEFINDER_CONFIG, 4); // Version incremented

PG_RESET_TEMPLATE(rangefinderConfig_t, rangefinderConfig,
    .rangefinder_hardware = SETTING_RANGEFINDER_HARDWARE_DEFAULT,
    .use_median_filtering = SETTING_RANGEFINDER_MEDIAN_FILTER_DEFAULT,
    .use_advanced_filtering = OFF,  // NEW
);
```

**settings.yaml**:
```yaml
  - name: PG_RANGEFINDER_CONFIG
    type: rangefinderConfig_t
    headers: ["sensors/rangefinder.h"]
    condition: USE_RANGEFINDER
    members:
      - name: rangefinder_hardware
        table: rangefinder_hardware
        description: "Selection of rangefinder hardware."
        default_value: "NONE"
      - name: rangefinder_median_filter
        description: "3-point median filtering for rangefinder readouts"
        default_value: OFF
        field: use_median_filtering
        type: bool
      - name: rangefinder_advanced_filter  # NEW
        description: "Advanced filtering for rangefinder"
        default_value: OFF
        field: use_advanced_filtering
        type: bool
```

### Example 2: Adding a Numeric Parameter

**Header file**:
```c
typedef struct rangefinderConfig_s {
    uint8_t rangefinder_hardware;
    uint8_t use_median_filtering;
    uint16_t max_range_cm;  // NEW
} rangefinderConfig_t;
```

**Source file**:
```c
PG_RESET_TEMPLATE(rangefinderConfig_t, rangefinderConfig,
    .rangefinder_hardware = SETTING_RANGEFINDER_HARDWARE_DEFAULT,
    .use_median_filtering = SETTING_RANGEFINDER_MEDIAN_FILTER_DEFAULT,
    .max_range_cm = 500,  // NEW
);
```

**settings.yaml**:
```yaml
      - name: rangefinder_max_range
        description: "Maximum rangefinder range in centimeters"
        default_value: 500
        field: max_range_cm
        min: 10
        max: 2000
```

### Example 3: Adding an Enum/Table Parameter

**Header file**:
```c
typedef enum {
    FILTER_MODE_NONE = 0,
    FILTER_MODE_LOW = 1,
    FILTER_MODE_MEDIUM = 2,
    FILTER_MODE_HIGH = 3,
} rangefinderFilterMode_e;

typedef struct rangefinderConfig_s {
    uint8_t rangefinder_hardware;
    uint8_t use_median_filtering;
    uint8_t filter_mode;  // NEW
} rangefinderConfig_t;
```

**Source file**:
```c
PG_RESET_TEMPLATE(rangefinderConfig_t, rangefinderConfig,
    .rangefinder_hardware = SETTING_RANGEFINDER_HARDWARE_DEFAULT,
    .use_median_filtering = SETTING_RANGEFINDER_MEDIAN_FILTER_DEFAULT,
    .filter_mode = FILTER_MODE_LOW,  // NEW
);
```

**settings.yaml** - Add table first:
```yaml
tables:
  - name: rangefinder_filter_mode
    values: ["NONE", "LOW", "MEDIUM", "HIGH"]
    enum: rangefinderFilterMode_e
```

Then add the parameter:
```yaml
      - name: rangefinder_filter_mode
        description: "Rangefinder filtering mode"
        default_value: LOW
        field: filter_mode
        table: rangefinder_filter_mode
```

## Creating a New Parameter Group

If you need to create a completely new configuration group:

1. **Create header file** (e.g., `src/main/custom/my_feature.h`):

```c
#pragma once

#include <stdint.h>
#include "config/parameter_group.h"

typedef struct myFeatureConfig_s {
    uint8_t enabled;
    uint16_t threshold;
} myFeatureConfig_t;

PG_DECLARE(myFeatureConfig_t, myFeatureConfig);
```

2. **Create source file** (e.g., `src/main/custom/my_feature.c`):

```c
#include "my_feature.h"

PG_REGISTER_WITH_RESET_TEMPLATE(myFeatureConfig_t, myFeatureConfig, PG_MY_FEATURE_CONFIG, 0);

PG_RESET_TEMPLATE(myFeatureConfig_t, myFeatureConfig,
    .enabled = OFF,
    .threshold = 100,
);
```

3. **Add ID to `parameter_group_ids.h`**:

```c
#define PG_MY_FEATURE_CONFIG 1045
#define PG_INAV_END PG_MY_FEATURE_CONFIG
```

4. **Add to `settings.yaml`**:

```yaml
  - name: PG_MY_FEATURE_CONFIG
    type: myFeatureConfig_t
    headers: ["custom/my_feature.h"]
    condition: USE_MY_FEATURE
    members:
      - name: my_feature_enabled
        description: "Enable my custom feature"
        default_value: OFF
        field: enabled
        type: bool
      - name: my_feature_threshold
        description: "Threshold value for my feature"
        default_value: 100
        field: threshold
        min: 0
        max: 1000
```

## Parameter Types in settings.yaml

- `bool` - Boolean (ON/OFF)
- `uint8_t` - 8-bit unsigned integer (0-255)
- `uint16_t` - 16-bit unsigned integer (0-65535)
- `uint32_t` - 32-bit unsigned integer
- `int8_t` - 8-bit signed integer
- `int16_t` - 16-bit signed integer
- `float` - Floating point number

## Accessing Configuration in Code

```c
#include "sensors/rangefinder.h"

// Read a parameter
if (rangefinderConfig()->use_median_filtering) {
    // Do something
}

uint16_t maxRange = rangefinderConfig()->max_range_cm;
```

## CLI Usage

After building, the parameters will be available in the CLI:

```
# Get current value
get rangefinder_median_filter

# Set value
set rangefinder_median_filter = ON

# Save to EEPROM
save
```

## Important Notes

1. **Version Numbers**: Increment the version number (4th parameter in `PG_REGISTER_WITH_RESET_TEMPLATE`) whenever you modify the structure
2. **Field Names**: The `field` property in `settings.yaml` must match the exact struct member name
3. **Default Values**: Must match between source file and `settings.yaml`
4. **Rebuild**: Always rebuild completely after modifying `settings.yaml`
5. **Testing**: Test with `get` and `set` commands in CLI before assuming it works

## Troubleshooting

- **Parameter not showing in CLI**: Check that the condition (e.g., `USE_RANGEFINDER`) is defined in your build
- **Build errors**: Ensure header paths in `settings.yaml` are correct
- **Values not persisting**: Make sure you call `save` after setting parameters
- **Default values wrong**: Check both `PG_RESET_TEMPLATE` and `settings.yaml` default values match

## References

- [src/main/fc/settings.yaml](src/main/fc/settings.yaml) - Main settings definition file
- [src/main/config/parameter_group_ids.h](src/main/config/parameter_group_ids.h) - Parameter group IDs
- Existing parameter groups for examples:
  - Rangefinder: [src/main/sensors/rangefinder.h](src/main/sensors/rangefinder.h) and [.c](src/main/sensors/rangefinder.c)
  - Gyro: [src/main/sensors/gyro.h](src/main/sensors/gyro.h) and [.c](src/main/sensors/gyro.c)
  - Navigation: Look for `PG_NAV_CONFIG` in the codebase
