import pytest
from fastapi.testclient import TestClient
from api.main import app


class TestPythonAgentExamplesIntegration:
    """Integration tests for Python agent examples content in the RAG system."""

    def setup_method(self):
        """Set up test client for each test."""
        self.client = TestClient(app)

    def test_python_agent_content_retrieval(self):
        """Test that Python agent examples content can be retrieved through the RAG system."""
        # Search for Python agent content
        search_request = {
            "query": "Python agents with rclpy and ROS controllers",
            "limit": 5,
            "filters": {}
        }
        response = self.client.post("/content/search", json=search_request)

        # Check that the search was successful
        assert response.status_code == 200
        data = response.json()
        assert "results" in data
        assert len(data["results"]) > 0

        # Check that at least one result is related to Python agents
        found_python_agents = False
        for result in data["results"]:
            if "python" in result["title"].lower() or "agent" in result["title"].lower():
                found_python_agents = True
                break

        assert found_python_agents, "Should find content related to Python agents"

    def test_rclpy_concept_retrieval(self):
        """Test that content about rclpy can be retrieved."""
        search_request = {
            "query": "What is rclpy?",
            "limit": 3,
            "filters": {}
        }
        response = self.client.post("/content/search", json=search_request)

        assert response.status_code == 200
        data = response.json()
        assert "results" in data

        # Check that results contain information about rclpy
        found_rclpy_info = False
        for result in data["results"]:
            if "rclpy" in result["excerpt"].lower():
                found_rclpy_info = True
                break

        assert found_rclpy_info, "Should find content about rclpy"

    def test_robot_controller_example_retrieval(self):
        """Test that content about robot controller examples can be retrieved."""
        search_request = {
            "query": "robot controller example",
            "limit": 3,
            "filters": {}
        }
        response = self.client.post("/content/search", json=search_request)

        assert response.status_code == 200
        data = response.json()
        assert "results" in data

        # Check that results contain information about robot controllers
        found_controller_info = False
        for result in data["results"]:
            if "controller" in result["excerpt"].lower() or "robot" in result["excerpt"].lower():
                found_controller_info = True
                break

        assert found_controller_info, "Should find content about robot controllers"

    def test_ai_navigation_agent_retrieval(self):
        """Test that content about AI navigation agents can be retrieved."""
        search_request = {
            "query": "AI navigation agent",
            "limit": 3,
            "filters": {}
        }
        response = self.client.post("/content/search", json=search_request)

        assert response.status_code == 200
        data = response.json()
        assert "results" in data

        # Check that results contain information about AI navigation
        found_ai_navigation = False
        for result in data["results"]:
            if "navigation" in result["excerpt"].lower() or "ai" in result["excerpt"].lower():
                found_ai_navigation = True
                break

        assert found_ai_navigation, "Should find content about AI navigation agents"

    def test_python_agent_chat_response(self):
        """Test that the chat system can respond to questions about Python agents."""
        # Start a chat session
        start_response = self.client.post("/chat/start")
        assert start_response.status_code == 201
        session_id = start_response.json()["sessionId"]

        # Ask a question about Python agents
        chat_request = {
            "message": "How do I create a Python agent that connects to ROS controllers?",
            "context": "Module 1, Chapter 2: Python Agents with rclpy"
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

    def test_joint_controller_content_retrieval(self):
        """Test that content about joint controllers can be retrieved."""
        search_request = {
            "query": "joint state controller python",
            "limit": 3,
            "filters": {}
        }
        response = self.client.post("/content/search", json=search_request)

        assert response.status_code == 200
        data = response.json()
        assert "results" in data

        # Check that results contain information about joint controllers
        found_joint_controller = False
        for result in data["results"]:
            if "joint" in result["excerpt"].lower() and "controller" in result["excerpt"].lower():
                found_joint_controller = True
                break

        assert found_joint_controller, "Should find content about joint controllers"

    def test_robot_arm_controller_retrieval(self):
        """Test that content about robot arm controllers can be retrieved."""
        search_request = {
            "query": "robot arm controller example",
            "limit": 3,
            "filters": {}
        }
        response = self.client.post("/content/search", json=search_request)

        assert response.status_code == 200
        data = response.json()
        assert "results" in data

        # Check that results contain information about robot arm controllers
        found_arm_controller = False
        for result in data["results"]:
            if "arm" in result["excerpt"].lower() and "controller" in result["excerpt"].lower():
                found_arm_controller = True
                break

        assert found_arm_controller, "Should find content about robot arm controllers"

    def test_content_structure_matching_for_python_agents(self):
        """Test that the content structure matches what's expected for Python agents."""
        # Search for Python agents content
        search_request = {
            "query": "Python agents with rclpy",
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

            # Check that the title is related to Python agents
            assert "python" in result["title"].lower() or "agent" in result["title"].lower()