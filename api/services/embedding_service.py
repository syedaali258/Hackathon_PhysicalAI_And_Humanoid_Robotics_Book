import asyncio
from typing import List, Dict, Any
from api.models.document import BookContent, ContentChunk
from api.services.vector_store import vector_db
import hashlib
import re


class EmbeddingService:
    def __init__(self):
        self.chunk_size = 500  # characters per chunk
        self.overlap = 50  # overlap between chunks

    def chunk_content(self, content: BookContent) -> List[ContentChunk]:
        """
        Split content into chunks for RAG retrieval.
        This is a simplified implementation - in a real system, you'd use
        proper NLP techniques to chunk semantically coherent pieces.
        """
        text = content.content
        chunks = []
        start = 0

        while start < len(text):
            # Find the end of the chunk, trying to break at sentence boundaries
            end = start + self.chunk_size

            if end >= len(text):
                end = len(text)
            else:
                # Try to find a sentence boundary near the end
                temp_end = end
                while temp_end < len(text) and temp_end > start + self.chunk_size // 2:
                    if text[temp_end] in '.!?':
                        end = temp_end + 1
                        break
                    temp_end += 1

            # If no sentence boundary found, try to break at word boundary
            if end == start + self.chunk_size:
                temp_end = end
                while temp_end < len(text) and temp_end > start + self.chunk_size // 2:
                    if text[temp_end] == ' ':
                        end = temp_end
                        break
                    temp_end -= 1

            chunk_text = text[start:end].strip()

            if len(chunk_text) > 0:  # Only add non-empty chunks
                chunk_id = hashlib.md5(f"{content.id}_{start}_{end}".encode()).hexdigest()

                chunk = ContentChunk(
                    id=chunk_id,
                    content_id=content.id,
                    text=chunk_text,
                    chunk_index=len(chunks),
                    metadata={
                        "source_title": content.title,
                        "source_type": content.type,
                        "source_slug": content.slug
                    }
                )
                chunks.append(chunk)

            # Move start forward with overlap
            start = max(end - self.overlap, end)  # Ensure we move forward even if overlap is larger than remaining text

        return chunks

    async def process_content(self, content: BookContent):
        """
        Process content by chunking it and storing in vector database.
        """
        chunks = self.chunk_content(content)

        # Prepare chunks for vector store
        vector_chunks = []
        for chunk in chunks:
            vector_chunks.append({
                "content_id": chunk.content_id,
                "text": chunk.text,
                "source": f"{chunk.metadata['source_title']} ({chunk.metadata['source_slug']})",
                "metadata": chunk.metadata
            })

        # Add to vector store
        vector_db.add_content_chunks(vector_chunks)

        return chunks

    async def process_content_batch(self, contents: List[BookContent]):
        """
        Process multiple content items.
        """
        all_chunks = []
        for content in contents:
            chunks = await self.process_content(content)
            all_chunks.extend(chunks)
        return all_chunks


# Singleton instance
embedding_service = EmbeddingService()