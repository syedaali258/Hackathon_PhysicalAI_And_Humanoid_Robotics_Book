# VLA Module API Documentation

This document provides comprehensive documentation for the Vision-Language-Action (VLA) module API endpoints.

## Base URL

All VLA API endpoints are prefixed with `/vla` and are part of the main API service.

## Authentication

The VLA API endpoints follow the same authentication scheme as the main API. For local development, no authentication is required. In production, API keys may be required for rate limiting.

## Endpoints

### Process Voice Command

`POST /vla/voice-command`

Process a voice command and initiate the VLA pipeline.

#### Request Body

```json
{
  "audio_data": "base64_encoded_audio_string",
  "text_input": "alternative_text_input",
  "session_id": "optional_session_identifier"
}
```

#### Request Fields

- `audio_data` (string, optional): Base64 encoded audio data to be processed for speech recognition. Either `audio_data` or `text_input` must be provided.
- `text_input` (string, optional): Alternative text input if audio is not available. Either `audio_data` or `text_input` must be provided.
- `session_id` (string, optional): Session identifier for maintaining conversation context.

#### Response

```json
{
  "pipeline_id": "vla_1634567890_12345",
  "recognized_text": "move forward",
  "confidence": 0.95,
  "status": "processing_speech"
}
```

#### Response Fields

- `pipeline_id` (string): Unique identifier for the VLA pipeline execution
- `recognized_text` (string): Text recognized from the audio input
- `confidence` (number): Confidence score for the speech recognition (0.0-1.0)
- `status` (string): Current status of the pipeline (pending, processing_speech, generating_plan, executing_actions, completed, failed)

#### Example Request

```bash
curl -X POST http://localhost:8000/vla/voice-command \
  -H "Content-Type: application/json" \
  -d '{
    "audio_data": "base64_encoded_audio_data_here",
    "session_id": "session_123"
  }'
```

---

### Generate Plan

`POST /vla/plan`

Generate a language-based plan for the given command using LLMs.

#### Request Body

```json
{
  "command_id": "vc_pipeline_123",
  "context": {
    "environment": "kitchen",
    "robot_state": "at_location",
    "objects": ["cup", "table"]
  }
}
```

#### Request Fields

- `command_id` (string, required): ID of the voice command to generate a plan for
- `context` (object, optional): Additional context information for plan generation

#### Response

```json
{
  "plan_id": "plan_1634567890_67890",
  "action_sequence": [
    {
      "action": "navigate_to",
      "parameters": {
        "target": "kitchen"
      }
    },
    {
      "action": "detect_object",
      "parameters": {
        "type": "cup",
        "color": "red"
      }
    }
  ],
  "interpreted_meaning": "User wants robot to navigate to kitchen and locate a red cup"
}
```

#### Response Fields

- `plan_id` (string): Unique identifier for the generated plan
- `action_sequence` (array): List of actions in the plan
- `interpreted_meaning` (string): LLM's interpretation of the command

---

### Execute Action

`POST /vla/execute-action`

Execute a robot action in the simulation environment.

#### Request Body

```json
{
  "action": {
    "id": "action_123",
    "action_type": "move_forward",
    "parameters": {
      "distance": 1.0,
      "speed": 0.5
    }
  },
  "environment_id": "env_kitchen_001"
}
```

#### Request Fields

- `action` (object, required): The robot action to execute
  - `id` (string): Unique action identifier
  - `action_type` (string): Type of action (move_forward, turn_left, etc.)
  - `parameters` (object): Action-specific parameters
- `environment_id` (string, required): ID of the simulation environment

#### Response

```json
{
  "action_id": "action_123",
  "execution_status": "completed",
  "timestamp": "2025-12-22T10:30:00Z"
}
```

#### Response Fields

- `action_id` (string): ID of the executed action
- `execution_status` (string): Status of the action execution
- `timestamp` (string): ISO timestamp of completion

---

### Execute Complete VLA Pipeline

`POST /vla/pipeline`

Execute the complete VLA pipeline: Vision, Language, Action.

#### Request Body

```json
{
  "command": "Navigate to the kitchen and find a red cup",
  "audio_data": "base64_encoded_audio_if_available",
  "environment_context": {
    "location": "living_room",
    "objects": [
      {"type": "chair", "position": [1.0, 2.0]},
      {"type": "table", "position": [3.0, 1.0]}
    ]
  }
}
```

#### Request Fields

- `command` (string, required): Natural language command to process
- `audio_data` (string, optional): Base64 encoded audio data
- `environment_context` (object, optional): Context about the environment

#### Response

```json
{
  "pipeline_id": "pipeline_1634567890_11111",
  "status": "completed",
  "actions": [
    {
      "id": "action_1",
      "action_type": "navigate",
      "parameters": {"target": "kitchen"}
    }
  ],
  "execution_log": [
    "Recognized command: Navigate to the kitchen and find a red cup",
    "Generated plan with 3 actions",
    "All actions executed successfully"
  ]
}
```

#### Response Fields

- `pipeline_id` (string): Unique identifier for the pipeline
- `status` (string): Current pipeline status
- `actions` (array): List of robot actions generated
- `execution_log` (array): Log of pipeline execution steps

---

### Get Pipeline Status

`GET /vla/status/{pipeline_id}`

Get the status of a VLA pipeline execution.

#### Path Parameters

- `pipeline_id` (string, required): ID of the pipeline to query

#### Response

```json
{
  "pipeline_id": "pipeline_123",
  "status": "completed",
  "actions": [],
  "execution_log": ["Pipeline completed successfully"]
}
```

---

### Health Check

`GET /vla/health`

Health check for the VLA module.

#### Response

```json
{
  "status": "healthy",
  "timestamp": "2025-12-22T10:30:00Z",
  "module": "VLA (Vision-Language-Action)"
}
```

## Common Response Codes

- `200 OK`: Request processed successfully
- `400 Bad Request`: Invalid request parameters
- `404 Not Found`: Requested resource not found
- `422 Unprocessable Entity`: Request validation failed
- `500 Internal Server Error`: Server error occurred

## Error Responses

When an error occurs, the API returns a JSON object with an error message:

```json
{
  "detail": "Error message describing the issue"
}
```

## Data Models

### VoiceCommand

Represents a spoken instruction from the user.

```json
{
  "id": "vc_123",
  "audio_data": "base64_encoded_audio",
  "transcribed_text": "move forward",
  "timestamp": "2025-12-22T10:30:00Z",
  "confidence": 1.0
}
```

### LanguagePlan

Contains the interpreted meaning and action sequence generated by the LLM.

```json
{
  "id": "plan_123",
  "command_id": "vc_123",
  "interpreted_meaning": "Move the robot forward by 1 meter",
  "action_sequence": [
    {
      "action": "move_forward",
      "parameters": {
        "distance": 1.0,
        "speed": 0.5
      }
    }
  ],
  "generated_by_llm": "gpt-4-turbo",
  "timestamp": "2025-12-22T10:30:00Z"
}
```

### RobotAction

Low-level control commands for the simulated robot.

```json
{
  "id": "action_123",
  "action_type": "move_forward",
  "parameters": {
    "distance": 1.0,
    "speed": 0.5
  },
  "execution_priority": 0,
  "estimated_duration_ms": 2000
}
```

### VLAPipeline

Represents a complete VLA pipeline execution.

```json
{
  "id": "pipeline_123",
  "voice_command": { /* VoiceCommand object */ },
  "language_plan": { /* LanguagePlan object */ },
  "robot_actions": [ /* Array of RobotAction objects */ ],
  "simulation_environment": { /* SimulationEnvironment object */ },
  "status": "completed",
  "created_at": "2025-12-22T10:30:00Z",
  "completed_at": "2025-12-22T10:31:00Z",
  "error_message": null
}
```

## Rate Limiting

The VLA API endpoints are subject to the same rate limiting as the main API. Exceeding the rate limit will result in a `429 Too Many Requests` response.

## Examples

### Processing a Voice Command

```bash
curl -X POST http://localhost:8000/vla/voice-command \
  -H "Content-Type: application/json" \
  -d '{
    "text_input": "move forward 2 meters",
    "session_id": "session_456"
  }'
```

### Executing a Complete Pipeline

```bash
curl -X POST http://localhost:8000/vla/pipeline \
  -H "Content-Type: application/json" \
  -d '{
    "command": "Go to the kitchen and find the red cup",
    "environment_context": {
      "current_location": "living_room"
    }
  }'
```

## Client Libraries

For implementation examples in different programming languages, refer to the [Client Libraries](/reference/client-libraries) section.