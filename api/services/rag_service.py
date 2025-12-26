import asyncio
from typing import List, Dict, Any
from api.models.chat import ChatRequest, ChatResponse
from api.services.openai_service import openai_service
from api.services.vector_store import vector_db
from api.services.embedding_service import embedding_service
from api.models.document import BookContent
from api.utils.errors import RAGException, ContentNotFoundException
from api.utils.logging import logger
import numpy as np
import re
from functools import lru_cache
import hashlib


class RAGService:
    def __init__(self):
        self.openai_service = openai_service
        self.vector_db = vector_db
        self.embedding_service = embedding_service
        # Simple in-memory cache for frequently accessed content
        self._cache = {}
        self._cache_ttl = 300  # 5 minutes

    def validate_query(self, query: str) -> bool:
        """
        Validate the user query for safety and appropriateness.
        """
        if not query or not query.strip():
            raise RAGException("Query cannot be empty")

        # Check for excessively long queries
        if len(query) > 1000:
            raise RAGException("Query is too long. Please keep it under 1000 characters.")

        # Check for potentially harmful patterns
        harmful_patterns = [
            r"(system|admin|root|sudo|exec|execute|run|shell|command)",
            r"(prompt|instruct|tell|ask).*ignore",
            r"(prompt|instruct|tell|ask).*previous",
            r"(\b\w+\b\s*){50,}"  # Too many repeated words
        ]

        for pattern in harmful_patterns:
            if re.search(pattern, query, re.IGNORECASE):
                raise RAGException("Query contains potentially harmful content.")

        return True

    def _get_cache_key(self, query: str, limit: int) -> str:
        """Generate a cache key for the given query and limit."""
        cache_input = f"{query}:{limit}"
        return hashlib.md5(cache_input.encode()).hexdigest()

    def _is_cache_valid(self, key: str) -> bool:
        """Check if cache entry is still valid (not expired)."""
        if key not in self._cache:
            return False

        cached_time = self._cache[key]["timestamp"]
        current_time = __import__('time').time()
        return (current_time - cached_time) < self._cache_ttl

    async def get_relevant_content(self, query: str, limit: int = 5) -> List[Dict[str, Any]]:
        """
        Retrieve relevant content chunks based on the query.
        This is a simplified implementation - in a real system, you would:
        1. Generate an embedding for the query
        2. Search the vector database for similar content
        3. Return the most relevant chunks
        """
        try:
            # Validate the query first
            self.validate_query(query)

            # Check cache first
            cache_key = self._get_cache_key(query, limit)
            if self._is_cache_valid(cache_key):
                logger.info(f"Cache hit for query: {query[:50]}...")
                return self._cache[cache_key]["data"]

            # For now, return empty results as a placeholder
            # In a real implementation, we would:
            # 1. Create an embedding for the query
            # 2. Search the vector database
            # 3. Return relevant content chunks

            # Placeholder implementation - in real system we would search vector DB
            # For now, return an empty list
            # In a real implementation, we would create a query embedding and search
            if self.vector_db is not None:
                query_embedding = [0.0] * 1536  # Placeholder embedding size
                results = self.vector_db.search_similar(query_embedding, limit=limit)
            else:
                # Return empty results if vector DB is not available
                results = []

            # Cache the results
            self._cache[cache_key] = {
                "data": results,
                "timestamp": __import__('time').time()
            }

            # Log the search results for debugging and analytics
            logger.info(
                f"Content search completed",
                extra={
                    "query": query[:100] + "..." if len(query) > 100 else query,
                    "result_count": len(results),
                    "content_sources": list(set([r.get("metadata", {}).get("source_title", "Unknown") for r in results])),
                    "cache_hit": False
                }
            )

            return results
        except RAGException:
            # Re-raise validation errors
            raise
        except Exception as e:
            logger.error(
                f"Error retrieving relevant content",
                extra={
                    "query": query[:100] + "..." if len(query) > 100 else query,
                    "error": str(e)
                },
                exc_info=True
            )
            raise RAGException(f"Error retrieving content: {str(e)}")

    async def generate_response(self, chat_request: ChatRequest, session_id: str = None) -> ChatResponse:
        """
        Generate a response using RAG (Retrieval Augmented Generation).
        """
        try:
            # Log the request
            logger.info(
                f"Processing chat request",
                extra={
                    "session_id": session_id,
                    "message": chat_request.message[:100] + "..." if len(chat_request.message) > 100 else chat_request.message
                }
            )

            # Validate the request
            self.validate_query(chat_request.message)

            # Get relevant content based on the query
            relevant_chunks = await self.get_relevant_content(chat_request.message)

            # Generate response using OpenAI with the relevant content
            if self.openai_service is not None:
                response_data = self.openai_service.generate_response(
                    user_message=chat_request.message,
                    context_chunks=relevant_chunks
                )
            else:
                # Return a mock response if OpenAI service is not available
                response_data = {
                    "response": "Mock response: The AI service is not available. In a real implementation, this would generate a response based on the provided context.",
                    "sources": relevant_chunks,
                    "confidence": 0.5
                }

            # Create and return the response
            response = ChatResponse(
                response=response_data["response"],
                sources=response_data["sources"],
                follow_up=response_data.get("follow_up", []),
                timestamp=__import__('datetime').datetime.now(),
                confidence=response_data.get("confidence", 0.8)
            )

            # Log successful response
            logger.info(
                f"Generated response successfully",
                extra={
                    "session_id": session_id,
                    "response_length": len(response.response)
                }
            )

            return response

        except RAGException:
            # Re-raise validation errors
            raise
        except Exception as e:
            logger.error(
                f"Error generating RAG response",
                extra={
                    "session_id": session_id,
                    "error": str(e)
                },
                exc_info=True
            )
            raise RAGException(f"Error generating response: {str(e)}")

    async def add_content(self, content: BookContent):
        """
        Add book content to the RAG system by chunking and storing in vector database.
        """
        try:
            # Validate content
            if not content or not content.content.strip():
                raise RAGException("Content cannot be empty")

            # Process the content (chunk and store in vector DB)
            chunks = await self.embedding_service.process_content(content)

            # Only store in vector database if it's available
            if self.vector_db is not None:
                # In a real implementation, we would store the chunks in the vector DB
                # self.vector_db.add_content_chunks(chunks)
                pass  # Placeholder - would store chunks in vector DB in real implementation

            logger.info(
                f"Added content to RAG system",
                extra={
                    "content_id": content.id,
                    "title": content.title,
                    "chunk_count": len(chunks)
                }
            )

            return chunks
        except RAGException:
            # Re-raise validation errors
            raise
        except Exception as e:
            logger.error(
                f"Error adding content to RAG system",
                extra={
                    "content_id": content.id,
                    "error": str(e)
                },
                exc_info=True
            )
            raise RAGException(f"Error adding content: {str(e)}")


# Singleton instance
rag_service = RAGService()