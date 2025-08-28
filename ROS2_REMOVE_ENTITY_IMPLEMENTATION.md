# Gazebo Remove Entity Implementation

## Overview

This document describes the implementation of the `makeRemoveEntity` function in `gz_ros2_srv.js` that converts the WoT action call for removing entities into a Gazebo service call. The implementation uses Gazebo services directly and leverages the existing world detection mechanism from `gz_actions.js`.

## Changes Made

### 1. Updated `gz_actions.js`
- **Exported `entityExists` function**: Added `entityExists` to the module exports to allow it to be used by the new ROS2 service function.

### 2. Enhanced `gz_ros2_srv.js`
- **Added `makeRemoveEntity` function**: Created a new factory function that follows the same pattern as `makeSetRtf`
- **Gazebo service integration**: Uses `gz service` command directly since no ROS2 bridge is running
- **World detection**: Uses the existing `get_world()` function from `gz_actions.js`
- **Input validation**: Validates that the entity name is provided
- **Error handling**: Comprehensive error handling with meaningful error messages

### 3. Updated `test_gz_controller.js`
- **Imported `makeRemoveEntity`**: Added the new function to the imports
- **Registered action handler**: Added `removeEntityRos2` action handler using the new function

### 4. Updated `gz_controller.json` (Thing Description)
- **Added `removeEntityRos2` action**: Defined the new action with proper input/output schema
- **Input schema**: Requires a `name` property of type string
- **Output schema**: Returns a confirmation message as string

## Function Signature

```javascript
function makeRemoveEntity(node, { debug = false, timeoutMs = 1000 } = {})
```

### Parameters
- `node`: The rclnodejs Node instance
- `debug`: Boolean flag for debug logging (default: false)
- `timeoutMs`: Service call timeout in milliseconds (default: 1000)

### Returns
A function that takes a WoT input object and returns a Promise that resolves to a confirmation message.

## Usage Example

```javascript
const { makeRemoveEntity } = require('./library/gazebo/gz_ros2_srv');

// Create the function
const removeEntity = makeRemoveEntity(node, { debug: true, timeoutMs: 2000 });

// Use it with WoT input
const input = {
  value: async () => ({ name: "my_entity" })
};

const result = await removeEntity(input);
console.log(result); // "my_entity has been removed."
```

## Service Details

The function uses the Gazebo service directly:

- **Service Command**: `gz service -s /world/{world}/remove --reqtype gz.msgs.Entity --reptype gz.msgs.Boolean`
- **Payload**: `name: "entity_name", type: 2`

The `type: 2` corresponds to `gz.msgs.Entity.MODEL` for model entities.

## World Detection Strategy

The function uses the existing `get_world()` function from `gz_actions.js`:

- **Automatic Detection**: The `get_world()` function automatically calls `detectWorldNames()` if no world is cached
- **Caching**: World names are cached after first detection for better performance
- **Consistency**: Uses the same world detection mechanism as other functions in the codebase

## Error Handling

The function handles various error scenarios:
- Missing entity name
- Entity not found in simulation (handled by Gazebo service)
- Service call failures
- Network timeouts

## Testing

A test script `test/test_remove_entity_ros2.js` has been created to verify the functionality. The test can be run with:

```bash
node test/test_remove_entity_ros2.js
```

## Integration

The new function is integrated into the existing WoT Thing and can be called via HTTP requests to the Thing's action endpoint:

```
POST /removeEntityRos2
Content-Type: application/json

{
  "name": "entity_to_remove"
}
```

This implementation maintains consistency with the existing codebase patterns and provides a robust, fallback-enabled solution for removing entities via ROS2 services.
