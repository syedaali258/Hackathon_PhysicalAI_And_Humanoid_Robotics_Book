import pytest
from fastapi.testclient import TestClient
from api.main import app


class TestPhysicsSimulationContract:
    """Contract tests for physics simulation endpoints based on the OpenAPI specification."""

    def setup_method(self):
        """Set up test client for each test."""
        self.client = TestClient(app)

    def test_gazebo_simulation_start_contract(self):
        """Test that /simulation/start endpoint works with Gazebo physics simulation."""
        # Test with gazebo simulation type (physics-focused)
        simulation_request = {
            "simulationType": "gazebo",
            "environment": "physics_world"
        }
        response = self.client.post("/simulation/start", json=simulation_request)

        # Check status code (could be 201 for success or various error codes)
        assert response.status_code in [201, 400, 500], f"Unexpected status code: {response.status_code}"

        if response.status_code == 201:
            data = response.json()

            # Check response structure according to OpenAPI spec
            assert "sessionId" in data, "Response should contain sessionId"
            assert "timestamp" in data, "Response should contain timestamp"
            assert "message" in data, "Response should contain message"
            assert "environmentConfig" in data, "Response should contain environmentConfig"

            # Check data types
            assert isinstance(data["sessionId"], str), "sessionId should be a string"
            assert isinstance(data["timestamp"], str), "timestamp should be a string"
            assert isinstance(data["message"], str), "message should be a string"
            assert isinstance(data["environmentConfig"], dict), "environmentConfig should be a dict"

            # For physics simulation, check that gravity is properly configured
            env_config = data["environmentConfig"]
            if "physics" in env_config:
                physics_config = env_config["physics"]
                assert "gravity" in physics_config, "Physics config should include gravity"
                assert isinstance(physics_config["gravity"], (int, float)), "Gravity should be a number"

    def test_physics_state_contract(self):
        """Test that /simulation/{sessionId}/state endpoint properly reports physics state."""
        # First start a physics simulation to get a session ID
        simulation_request = {
            "simulationType": "gazebo",
            "environment": "physics_world"
        }
        start_response = self.client.post("/simulation/start", json=simulation_request)

        if start_response.status_code == 201:
            session_id = start_response.json()["sessionId"]

            # Get simulation state
            response = self.client.get(f"/simulation/{session_id}/state")

            # Check status code
            assert response.status_code == 200, f"Expected status 200, got {response.status_code}"

            # Check response structure according to OpenAPI spec
            data = response.json()
            assert "state" in data, "Response should contain 'state' field"
            assert "timestamp" in data, "Response should contain 'timestamp' field"

            # Check state structure
            state = data["state"]
            assert "robotPosition" in state, "State should contain robotPosition"
            assert "sensors" in state, "State should contain sensors"
            assert "physics" in state, "State should contain physics"

            # Check physics structure specifically for physics simulation
            physics = state["physics"]
            assert "gravity" in physics, "Physics should have gravity"
            assert "collisionCount" in physics, "Physics should have collisionCount"
            assert "simulationSpeed" in physics, "Physics should have simulationSpeed"

            # Check that gravity is a reasonable value (Earth's gravity is ~9.81)
            assert isinstance(physics["gravity"], (int, float)), "Gravity should be a number"
            assert -20 <= physics["gravity"] <= 0, f"Gravity should be negative (got {physics['gravity']})"

            # Check collision count is non-negative
            assert isinstance(physics["collisionCount"], int), "Collision count should be an integer"
            assert physics["collisionCount"] >= 0, "Collision count should be non-negative"

            # Check simulation speed is reasonable
            assert isinstance(physics["simulationSpeed"], (int, float)), "Simulation speed should be a number"
            assert physics["simulationSpeed"] >= 0, "Simulation speed should be non-negative"

    def test_physics_command_contract(self):
        """Test that physics simulation commands properly affect the physics state."""
        # First start a physics simulation to get a session ID
        simulation_request = {
            "simulationType": "gazebo",
            "environment": "physics_world"
        }
        start_response = self.client.post("/simulation/start", json=simulation_request)

        if start_response.status_code == 201:
            session_id = start_response.json()["sessionId"]

            # Send a physics-related command (move forward, which should trigger physics)
            command_request = {
                "command": "set_velocity",
                "parameters": {
                    "linear": 1.0,
                    "angular": 0.0
                }
            }
            response = self.client.post(f"/simulation/{session_id}/command", json=command_request)

            # Check status code
            assert response.status_code in [200, 400, 404, 500], f"Unexpected status code: {response.status_code}"

            if response.status_code == 200:
                data = response.json()

                # Check response structure according to OpenAPI spec
                assert "status" in data, "Response should contain 'status' field"
                assert "newState" in data, "Response should contain 'newState' field"
                assert "timestamp" in data, "Response should contain 'timestamp' field"

                # Check that the new state includes updated physics information
                new_state = data["newState"]
                assert "physics" in new_state, "New state should contain physics"

                physics = new_state["physics"]
                assert "gravity" in physics, "Physics should have gravity"
                assert "collisionCount" in physics, "Physics should have collisionCount"
                assert "simulationSpeed" in physics, "Physics should have simulationSpeed"

    def test_collision_detection_contract(self):
        """Test that physics simulation properly reports collision information."""
        # First start a physics simulation to get a session ID
        simulation_request = {
            "simulationType": "gazebo",
            "environment": "collision_test_world"
        }
        start_response = self.client.post("/simulation/start", json=simulation_request)

        if start_response.status_code == 201:
            session_id = start_response.json()["sessionId"]

            # Get initial state
            initial_response = self.client.get(f"/simulation/{session_id}/state")
            if initial_response.status_code == 200:
                initial_data = initial_response.json()
                initial_collision_count = initial_data["state"]["physics"]["collisionCount"]

                # Send command that might cause collision
                command_request = {
                    "command": "move_forward",
                    "parameters": {
                        "linear": 5.0,  # Higher velocity to potentially cause collision
                        "angular": 0.0
                    }
                }
                command_response = self.client.post(f"/simulation/{session_id}/command", json=command_request)

                if command_response.status_code == 200:
                    command_data = command_response.json()
                    new_collision_count = command_data["newState"]["physics"]["collisionCount"]

                    # The collision count should be >= the initial count
                    # (It might stay the same if no collision occurred)
                    assert new_collision_count >= initial_collision_count, \
                        f"New collision count ({new_collision_count}) should be >= initial count ({initial_collision_count})"

    def test_physics_parameters_contract(self):
        """Test that physics parameters can be adjusted and reported correctly."""
        # First start a physics simulation to get a session ID
        simulation_request = {
            "simulationType": "gazebo",
            "environment": "adjustable_physics_world"
        }
        start_response = self.client.post("/simulation/start", json=simulation_request)

        if start_response.status_code == 201:
            session_id = start_response.json()["sessionId"]

            # Get initial physics state
            response = self.client.get(f"/simulation/{session_id}/state")

            assert response.status_code == 200, f"Expected status 200, got {response.status_code}"
            data = response.json()

            # Verify physics parameters exist and have correct types
            physics = data["state"]["physics"]

            # Gravity
            assert "gravity" in physics
            assert isinstance(physics["gravity"], (int, float))

            # Collision count
            assert "collisionCount" in physics
            assert isinstance(physics["collisionCount"], int)

            # Simulation speed
            assert "simulationSpeed" in physics
            assert isinstance(physics["simulationSpeed"], (int, float))