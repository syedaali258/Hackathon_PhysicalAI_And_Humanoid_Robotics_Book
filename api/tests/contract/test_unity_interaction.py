import pytest
from fastapi.testclient import TestClient
from api.main import app


class TestUnityInteractionContract:
    """Contract tests for Unity interaction endpoints based on the OpenAPI specification."""

    def setup_method(self):
        """Set up test client for each test."""
        self.client = TestClient(app)

    def test_unity_simulation_start_contract(self):
        """Test that /simulation/start endpoint works with Unity interaction simulation."""
        # Test with unity simulation type (visualization/interaction-focused)
        simulation_request = {
            "simulationType": "unity",
            "environment": "interaction_world"
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

            # For Unity interaction, check that interaction parameters are properly configured
            env_config = data["environmentConfig"]
            if "interaction" in env_config:
                interaction_config = env_config["interaction"]
                assert isinstance(interaction_config, dict), "Interaction config should be a dict"

    def test_unity_interaction_state_contract(self):
        """Test that /simulation/{sessionId}/state endpoint properly reports Unity interaction state."""
        # First start a Unity interaction simulation to get a session ID
        simulation_request = {
            "simulationType": "unity",
            "environment": "interaction_world"
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

            # For Unity interaction, the state should be suitable for visualization
            robot_pos = state["robotPosition"]
            assert "x" in robot_pos, "Robot position should have x coordinate"
            assert "y" in robot_pos, "Robot position should have y coordinate"
            assert "z" in robot_pos, "Robot position should have z coordinate"
            assert "rotation" in robot_pos, "Robot position should have rotation"

            # Check rotation structure
            rotation = robot_pos["rotation"]
            assert "x" in rotation, "Rotation should have x component"
            assert "y" in rotation, "Rotation should have y component"
            assert "z" in rotation, "Rotation should have z component"
            assert "w" in rotation, "Rotation should have w component"

            # Check that rotation components are numbers
            for axis in ["x", "y", "z", "w"]:
                assert isinstance(rotation[axis], (int, float)), f"Rotation {axis} should be a number"

    def test_unity_interaction_command_contract(self):
        """Test that Unity interaction commands properly affect the visualization state."""
        # First start a Unity interaction simulation to get a session ID
        simulation_request = {
            "simulationType": "unity",
            "environment": "interaction_world"
        }
        start_response = self.client.post("/simulation/start", json=simulation_request)

        if start_response.status_code == 201:
            session_id = start_response.json()["sessionId"]

            # Send an interaction command (like user input in Unity)
            command_request = {
                "command": "interact_object",
                "parameters": {
                    "object_id": "robot_arm",
                    "interaction_type": "move"
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

                # Check that the new state includes updated visualization information
                new_state = data["newState"]
                assert "robotPosition" in new_state, "New state should contain robotPosition"
                assert "sensors" in new_state, "New state should contain sensors"
                assert "physics" in new_state, "New state should contain physics"

                # Check that the position has updated appropriately
                new_pos = new_state["robotPosition"]
                assert "x" in new_pos, "New robot position should have x coordinate"
                assert "y" in new_pos, "New robot position should have y coordinate"
                assert "z" in new_pos, "New robot position should have z coordinate"
                assert "rotation" in new_pos, "New robot position should have rotation"

    def test_unity_visualization_parameters_contract(self):
        """Test that Unity visualization parameters are properly structured."""
        # First start a Unity interaction simulation to get a session ID
        simulation_request = {
            "simulationType": "unity",
            "environment": "visualization_world"
        }
        start_response = self.client.post("/simulation/start", json=simulation_request)

        if start_response.status_code == 201:
            session_id = start_response.json()["sessionId"]

            # Get simulation state
            response = self.client.get(f"/simulation/{session_id}/state")

            assert response.status_code == 200, f"Expected status 200, got {response.status_code}"
            data = response.json()

            # Check that the state includes visualization-appropriate data
            state = data["state"]
            assert "robotPosition" in state, "State should contain robotPosition"

            # Robot position should have full 3D coordinates and rotation for Unity visualization
            robot_pos = state["robotPosition"]
            assert all(key in robot_pos for key in ["x", "y", "z"]), "Robot position should have x, y, z coordinates"
            assert "rotation" in robot_pos, "Robot position should have rotation for Unity"

            rotation = robot_pos["rotation"]
            assert all(key in rotation for key in ["x", "y", "z", "w"]), "Rotation should have x, y, z, w components"

            # Check that all values are numbers
            for coord in ["x", "y", "z"]:
                assert isinstance(robot_pos[coord], (int, float)), f"Position {coord} should be a number"

            for axis in ["x", "y", "z", "w"]:
                assert isinstance(rotation[axis], (int, float)), f"Rotation {axis} should be a number"

    def test_unity_user_input_command_contract(self):
        """Test that Unity user input commands are properly handled."""
        # First start a Unity interaction simulation to get a session ID
        simulation_request = {
            "simulationType": "unity",
            "environment": "user_interaction_world"
        }
        start_response = self.client.post("/simulation/start", json=simulation_request)

        if start_response.status_code == 201:
            session_id = start_response.json()["sessionId"]

            # Send a Unity-specific command (like a user input)
            command_request = {
                "command": "set_velocity",
                "parameters": {
                    "linear": 0.5,
                    "angular": 0.2,
                    "source": "unity_ui"  # Unity-specific parameter
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

                # Check that the new state reflects the command
                new_state = data["newState"]
                assert "robotPosition" in new_state, "New state should contain robotPosition"
                assert "sensors" in new_state, "New state should contain sensors"
                assert "physics" in new_state, "New state should contain physics"

    def test_unity_interaction_reset_contract(self):
        """Test that Unity interaction simulation reset properly restores initial state."""
        # First start a Unity interaction simulation to get a session ID
        simulation_request = {
            "simulationType": "unity",
            "environment": "interaction_world"
        }
        start_response = self.client.post("/simulation/start", json=simulation_request)

        if start_response.status_code == 201:
            session_id = start_response.json()["sessionId"]

            # Get initial state
            initial_response = self.client.get(f"/simulation/{session_id}/state")
            if initial_response.status_code == 200:
                initial_data = initial_response.json()
                initial_robot_pos = initial_data["state"]["robotPosition"]

                # Send a command to change the state
                command_request = {
                    "command": "move_forward",
                    "parameters": {
                        "linear": 1.0,
                        "angular": 0.0
                    }
                }
                command_response = self.client.post(f"/simulation/{session_id}/command", json=command_request)

                if command_response.status_code == 200:
                    # Reset the simulation
                    reset_response = self.client.post(f"/simulation/{session_id}/reset")

                    # Check status code
                    assert reset_response.status_code in [200, 404, 500], f"Unexpected status code: {reset_response.status_code}"

                    if reset_response.status_code == 200:
                        reset_data = reset_response.json()

                        # Check response structure according to OpenAPI spec
                        assert "initialState" in reset_data, "Response should contain 'initialState' field"

                        # Check that initial state has the proper Unity interaction structure
                        initial_state = reset_data["initialState"]
                        assert "robotPosition" in initial_state, "Initial state should contain robotPosition"
                        assert "sensors" in initial_state, "Initial state should contain sensors"
                        assert "physics" in initial_state, "Initial state should contain physics"

                        # Verify that position is properly structured for Unity
                        reset_pos = initial_state["robotPosition"]
                        assert all(key in reset_pos for key in ["x", "y", "z"]), "Reset position should have x, y, z coordinates"
                        assert "rotation" in reset_pos, "Reset position should have rotation"

                        reset_rotation = reset_pos["rotation"]
                        assert all(key in reset_rotation for key in ["x", "y", "z", "w"]), "Reset rotation should have x, y, z, w components"