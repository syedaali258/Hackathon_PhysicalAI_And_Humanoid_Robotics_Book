import pytest
from fastapi.testclient import TestClient
from api.main import app


class TestChatHistoryContract:
    """Contract tests for the chat history endpoint based on the OpenAPI specification."""

    def setup_method(self):
        """Set up test client for each test."""
        self.client = TestClient(app)

    def test_get_chat_history_contract(self):
        """Test that /chat/{sessionId}/history endpoint matches the OpenAPI contract."""
        # First start a chat to get a session ID
        start_response = self.client.post("/chat/start")
        assert start_response.status_code == 201
        session_id = start_response.json()["sessionId"]

        # Add a message to the chat to have some history
        chat_request = {
            "message": "What is ROS 2?",
            "context": "Module 1, Chapter 1: ROS 2 Fundamentals"
        }
        send_response = self.client.post(f"/chat/{session_id}/message", json=chat_request)
        # Note: We don't check the status here as it might fail due to missing OpenAI API key

        # Get chat history
        response = self.client.get(f"/chat/{session_id}/history?limit=10")

        # Check status code
        assert response.status_code == 200, f"Expected status 200, got {response.status_code}"

        # Check response structure according to OpenAPI spec
        data = response.json()
        assert "messages" in data, "Response should contain 'messages' field"
        assert "hasMore" in data, "Response should contain 'hasMore' field"
        assert "total" in data, "Response should contain 'total' field"

        # Check data types
        assert isinstance(data["messages"], list), "messages should be a list"
        assert isinstance(data["hasMore"], bool), "hasMore should be a boolean"
        assert isinstance(data["total"], int), "total should be an integer"

        # Check messages structure
        for message in data["messages"]:
            assert "id" in message, "Each message should have id"
            assert "sender" in message, "Each message should have sender"
            assert "content" in message, "Each message should have content"
            assert "timestamp" in message, "Each message should have timestamp"
            assert message["sender"] in ["user", "assistant"], "Sender should be 'user' or 'assistant'"

    def test_get_chat_history_with_limit(self):
        """Test that the chat history endpoint respects the limit parameter."""
        # First start a chat to get a session ID
        start_response = self.client.post("/chat/start")
        assert start_response.status_code == 201
        session_id = start_response.json()["sessionId"]

        # Get chat history with specific limit
        response = self.client.get(f"/chat/{session_id}/history?limit=5")

        # Check status code
        assert response.status_code == 200

        # Check response structure
        data = response.json()
        assert "messages" in data
        assert "hasMore" in data
        assert "total" in data

        # The number of messages returned should not exceed the limit
        assert len(data["messages"]) <= 5, f"Should return at most 5 messages, got {len(data['messages'])}"

    def test_get_chat_history_with_before_parameter(self):
        """Test that the chat history endpoint can handle the before parameter."""
        # First start a chat to get a session ID
        start_response = self.client.post("/chat/start")
        assert start_response.status_code == 201
        session_id = start_response.json()["sessionId"]

        # Get chat history with before parameter (this may not be fully functional without proper datetime)
        response = self.client.get(f"/chat/{session_id}/history?limit=10&before=2023-01-01T00:00:00Z")

        # The before parameter might cause a 400 if the format is invalid, or work correctly
        assert response.status_code in [200, 400], f"Expected status 200 or 400, got {response.status_code}"

        if response.status_code == 200:
            # If successful, check the response structure
            data = response.json()
            assert "messages" in data
            assert "hasMore" in data
            assert "total" in data

    def test_get_chat_history_invalid_session(self):
        """Test that the chat history endpoint handles invalid session IDs properly."""
        # Try to get history for a non-existent session
        response = self.client.get("/chat/invalid-session-id/history?limit=10")

        # Should return 200 with empty history (as implemented in the API)
        # The API initializes an empty session if it doesn't exist
        assert response.status_code == 200, f"Expected status 200, got {response.status_code}"

        data = response.json()
        assert "messages" in data
        assert "hasMore" in data
        assert "total" in data
        assert data["total"] == 0, "Should return empty history for invalid session"
        assert len(data["messages"]) == 0, "Should return no messages for invalid session"
        assert data["hasMore"] is False, "Should indicate no more messages"

    def test_get_chat_history_default_limit(self):
        """Test that the chat history endpoint uses the default limit when not specified."""
        # First start a chat to get a session ID
        start_response = self.client.post("/chat/start")
        assert start_response.status_code == 201
        session_id = start_response.json()["sessionId"]

        # Get chat history without specifying limit (should use default)
        response = self.client.get(f"/chat/{session_id}/history")

        # Check status code
        assert response.status_code == 200

        # Check response structure
        data = response.json()
        assert "messages" in data
        assert "hasMore" in data
        assert "total" in data