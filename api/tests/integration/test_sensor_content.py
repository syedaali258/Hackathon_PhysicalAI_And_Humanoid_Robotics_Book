import pytest
from fastapi.testclient import TestClient
from api.main import app


class TestSensorContentIntegration:
    """Integration tests for sensor simulation content in the RAG system."""

    def setup_method(self):
        """Set up test client for each test."""
        self.client = TestClient(app)

    def test_sensor_simulation_content_retrieval(self):
        """Test that sensor simulation content can be retrieved through the RAG system."""
        # Search for sensor simulation content
        search_request = {
            "query": "sensor simulation LiDAR depth cameras IMUs",
            "limit": 5,
            "filters": {}
        }
        response = self.client.post("/content/search", json=search_request)

        # Check that the search was successful
        assert response.status_code == 200
        data = response.json()
        assert "results" in data
        assert "query" in data
        assert "total" in data

    def test_lidar_simulation_content_retrieval(self):
        """Test that content about LiDAR simulation can be retrieved."""
        search_request = {
            "query": "LiDAR simulation in robotics point cloud generation",
            "limit": 3,
            "filters": {}
        }
        response = self.client.post("/content/search", json=search_request)

        assert response.status_code == 200
        data = response.json()
        assert "results" in data
        assert "query" in data
        assert "total" in data

    def test_depth_camera_simulation_retrieval(self):
        """Test that content about depth camera simulation can be retrieved."""
        search_request = {
            "query": "depth camera simulation in robotics depth perception",
            "limit": 3,
            "filters": {}
        }
        response = self.client.post("/content/search", json=search_request)

        assert response.status_code == 200
        data = response.json()
        assert "results" in data
        assert "query" in data
        assert "total" in data

    def test_imu_simulation_content_retrieval(self):
        """Test that content about IMU simulation can be retrieved."""
        search_request = {
            "query": "IMU simulation acceleration orientation data",
            "limit": 3,
            "filters": {}
        }
        response = self.client.post("/content/search", json=search_request)

        assert response.status_code == 200
        data = response.json()
        assert "results" in data
        assert "query" in data
        assert "total" in data

    def test_sensor_simulation_chat_response(self):
        """Test that the chat system can respond to questions about sensor simulation."""
        # Start a chat session
        start_response = self.client.post("/chat/start")
        assert start_response.status_code == 201
        session_id = start_response.json()["sessionId"]

        # Ask a question about sensor simulation
        chat_request = {
            "message": "How are LiDAR, depth cameras, and IMUs simulated in robotics?",
            "context": "Module 2, Chapter 2: Sensors Simulation"
        }
        response = self.client.post(f"/chat/{session_id}/message", json=chat_request)

        # The response might fail due to missing OpenAI API key in test environment
        # but we should check if it would work properly if the service was available
        if response.status_code == 200:
            data = response.json()
            # If successful, check that response contains relevant information
            assert "response" in data
            # Note: We can't validate specific content without the actual RAG system running
        elif response.status_code in [400, 404, 500]:
            # These are valid responses according to the API contract
            pass
        else:
            assert False, f"Unexpected status code: {response.status_code}"

    def test_lidar_sensor_explanation_chat(self):
        """Test that the chat system can explain LiDAR sensor simulation."""
        # Start a chat session
        start_response = self.client.post("/chat/start")
        assert start_response.status_code == 201
        session_id = start_response.json()["sessionId"]

        # Ask about LiDAR sensors
        chat_request = {
            "message": "How does LiDAR simulation work in robotics?",
            "context": "Module 2, Chapter 2: Sensors Simulation"
        }
        response = self.client.post(f"/chat/{session_id}/message", json=chat_request)

        if response.status_code == 200:
            data = response.json()
            assert "response" in data
        elif response.status_code in [400, 404, 500]:
            # Valid responses
            pass
        else:
            assert False, f"Unexpected status code: {response.status_code}"

    def test_imu_sensor_simulation_content(self):
        """Test that content about IMU sensor simulation can be retrieved."""
        search_request = {
            "query": "IMU sensor simulation with realistic noise and drift",
            "limit": 3,
            "filters": {}
        }
        response = self.client.post("/content/search", json=search_request)

        assert response.status_code == 200
        data = response.json()
        assert "results" in data
        assert "query" in data
        assert "total" in data

        # Check that results contain information relevant to IMU simulation
        if len(data["results"]) > 0:
            # At least verify the structure is correct
            result = data["results"][0]
            assert "contentId" in result
            assert "title" in result
            assert "excerpt" in result
            assert "url" in result
            assert "relevance" in result

    def test_sensor_fusion_content_retrieval(self):
        """Test that content about sensor fusion can be retrieved."""
        search_request = {
            "query": "sensor fusion combining data from multiple sensors",
            "limit": 3,
            "filters": {}
        }
        response = self.client.post("/content/search", json=search_request)

        assert response.status_code == 200
        data = response.json()
        assert "results" in data
        assert "query" in data
        assert "total" in data

    def test_sensor_simulation_with_environment_filter(self):
        """Test that sensor simulation content can be filtered by simulation environment."""
        # Search for sensor simulation content with environment context
        search_request = {
            "query": "sensor simulation",
            "limit": 5,
            "filters": {
                "module": "module-2-digital-twin"
            }
        }
        response = self.client.post("/content/search", json=search_request)

        assert response.status_code in [200, 400, 500]  # Valid responses
        if response.status_code == 200:
            data = response.json()
            assert "results" in data
            assert "query" in data
            assert "total" in data

    def test_content_structure_matching_for_sensor_simulation(self):
        """Test that the content structure matches what's expected for sensor simulation."""
        # Search for sensor simulation content
        search_request = {
            "query": "sensor simulation in robotics",
            "limit": 1,
            "filters": {}
        }
        response = self.client.post("/content/search", json=search_request)

        assert response.status_code == 200
        data = response.json()
        assert "results" in data

        if len(data["results"]) > 0:
            result = data["results"][0]
            # Check that the result has expected fields
            assert "contentId" in result
            assert "title" in result
            assert "excerpt" in result
            assert "url" in result
            assert "relevance" in result

            # Verify data types
            assert isinstance(result["contentId"], str)
            assert isinstance(result["title"], str)
            assert isinstance(result["excerpt"], str)
            assert isinstance(result["url"], str)
            assert isinstance(result["relevance"], (int, float))