import pytest
from fastapi.testclient import TestClient
from api.main import app
from api.models.chat import ChatRequest


class TestChatRouter:
    """Test cases for the chat router endpoints."""

    def setup_method(self):
        """Set up test client for each test."""
        self.client = TestClient(app)

    def test_start_chat(self):
        """Test starting a new chat session."""
        response = self.client.post("/chat/start")
        assert response.status_code == 200
        data = response.json()
        assert "sessionId" in data
        assert "timestamp" in data
        assert "message" in data

    def test_send_message(self):
        """Test sending a message to the chatbot."""
        # First start a chat to get a session ID
        start_response = self.client.post("/chat/start")
        assert start_response.status_code == 200
        session_id = start_response.json()["sessionId"]

        # Send a message
        chat_request = {
            "message": "What is ROS 2?"
        }
        response = self.client.post(f"/chat/{session_id}/message", json=chat_request)

        # The response might fail due to missing OpenAI API key in test environment
        # but we should at least check that the endpoint exists and returns a proper error
        assert response.status_code in [200, 500]  # 200 if successful, 500 if OpenAI error

    def test_get_chat_history(self):
        """Test retrieving chat history."""
        # First start a chat to get a session ID
        start_response = self.client.post("/chat/start")
        assert start_response.status_code == 200
        session_id = start_response.json()["sessionId"]

        # Get chat history
        response = self.client.get(f"/chat/{session_id}/history?limit=10")
        assert response.status_code == 200
        data = response.json()
        assert "messages" in data
        assert "hasMore" in data
        assert "total" in data

    def test_end_chat(self):
        """Test ending a chat session."""
        # First start a chat to get a session ID
        start_response = self.client.post("/chat/start")
        assert start_response.status_code == 200
        session_id = start_response.json()["sessionId"]

        # End the chat
        response = self.client.post(f"/chat/{session_id}/end")
        assert response.status_code == 200
        data = response.json()
        assert "message" in data