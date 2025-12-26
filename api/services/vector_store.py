from qdrant_client import QdrantClient
from qdrant_client.http import models
from typing import List, Optional
import uuid
from pydantic import BaseModel


class VectorDB:
    def __init__(self, host: str = "localhost", port: int = 6333):
        """
        Initialize the Qdrant vector database client.
        For production, you might want to use Qdrant cloud or adjust settings.
        """
        try:
            # Try to connect to Qdrant server
            self.client = QdrantClient(host=host, port=port)
        except:
            # If connection fails, use in-memory storage for development
            self.client = QdrantClient(":memory:")

        self.collection_name = "book_content_chunks"
        self._initialize_collection()

    def _initialize_collection(self):
        """Initialize the collection for storing content chunks."""
        try:
            # Check if collection exists
            self.client.get_collection(self.collection_name)
        except:
            # Create collection if it doesn't exist
            self.client.create_collection(
                collection_name=self.collection_name,
                vectors_config=models.VectorParams(size=1536, distance=models.Distance.COSINE),
            )

    def add_content_chunks(self, chunks: List[dict]):
        """Add content chunks to the vector database."""
        points = []
        for chunk in chunks:
            point = models.PointStruct(
                id=str(uuid.uuid4()),
                vector=[0.0] * 1536,  # Placeholder - in real implementation, this would be the actual embedding
                payload={
                    "content_id": chunk.get("content_id", ""),
                    "text": chunk.get("text", ""),
                    "source": chunk.get("source", ""),
                    "metadata": chunk.get("metadata", {})
                }
            )
            points.append(point)

        self.client.upsert(collection_name=self.collection_name, points=points)

    def search_similar(self, query_vector: List[float], limit: int = 5) -> List[dict]:
        """Search for similar content chunks based on the query vector."""
        results = self.client.search(
            collection_name=self.collection_name,
            query_vector=query_vector,
            limit=limit
        )

        return [
            {
                "content_id": hit.payload.get("content_id"),
                "text": hit.payload.get("text"),
                "source": hit.payload.get("source"),
                "metadata": hit.payload.get("metadata"),
                "score": hit.score
            }
            for hit in results
        ]


# Singleton instance with error handling
try:
    vector_db = VectorDB()
except Exception as e:
    print(f"Warning: Could not initialize vector database: {e}. Using fallback functionality.")
    vector_db = None