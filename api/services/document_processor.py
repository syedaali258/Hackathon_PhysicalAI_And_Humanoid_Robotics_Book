"""
Document Processing Service for VLA Module Content
"""
import asyncio
import logging
import os
from typing import List, Dict, Any, Optional
from pathlib import Path
import hashlib
from datetime import datetime

from models.document import DocumentChunk, BookContent
from services.vector_store import vector_service

logger = logging.getLogger(__name__)


class VLADocumentProcessor:
    """
    Service for processing VLA module documentation content for RAG system
    """

    def __init__(self):
        self.vector_service = vector_service
        self.content_base_path = os.getenv("BOOK_PATH", "../book/docs")

    async def process_vla_content(self, module_path: Optional[str] = None) -> Dict[str, Any]:
        """
        Process all VLA module content for RAG system

        Args:
            module_path: Path to VLA module content (optional, defaults to standard location)

        Returns:
            Dictionary with processing results
        """
        if module_path is None:
            module_path = f"{self.content_base_path}/modules/04-vla-module"

        try:
            # Get all markdown files in the VLA module
            content_files = self._find_content_files(module_path)

            processed_count = 0
            processed_documents = []
            errors = []

            for file_path in content_files:
                try:
                    # Read and process each content file
                    file_content = await self._read_content_file(file_path)

                    # Create BookContent object
                    content_id = self._generate_content_id(file_path)
                    book_content = BookContent(
                        id=content_id,
                        title=self._extract_title(file_content),
                        body=file_content,
                        module="04-vla-module",
                        chapter=self._extract_chapter_from_path(file_path),
                        version="1.0.0",  # This would come from the actual documentation
                        last_updated=datetime.now(),
                        references=self._extract_references(file_content)
                    )

                    # Process and store in vector database
                    chunks = await self._chunk_and_store_content(book_content)

                    processed_count += 1
                    processed_documents.append(content_id)

                    logger.info(f"Processed VLA content: {file_path} ({len(chunks)} chunks)")

                except Exception as e:
                    error_msg = f"Error processing {file_path}: {str(e)}"
                    errors.append(error_msg)
                    logger.error(error_msg)

            return {
                "processed_count": processed_count,
                "processed_documents": processed_documents,
                "errors": errors,
                "total_files_found": len(content_files)
            }

        except Exception as e:
            logger.error(f"Error in VLA document processing: {str(e)}")
            return {
                "processed_count": 0,
                "processed_documents": [],
                "errors": [str(e)],
                "total_files_found": 0
            }

    def _find_content_files(self, module_path: str) -> List[str]:
        """
        Find all content files in the VLA module directory

        Args:
            module_path: Path to the VLA module

        Returns:
            List of content file paths
        """
        content_files = []
        path_obj = Path(module_path)

        # Find all markdown files in the module directory
        for file_path in path_obj.rglob("*.md"):
            content_files.append(str(file_path))

        return content_files

    async def _read_content_file(self, file_path: str) -> str:
        """
        Read content from a file

        Args:
            file_path: Path to the content file

        Returns:
            File content as string
        """
        loop = asyncio.get_event_loop()
        content = await loop.run_in_executor(None, self._sync_read_file, file_path)
        return content

    def _sync_read_file(self, file_path: str) -> str:
        """
        Synchronously read a file (for use with executor)
        """
        with open(file_path, 'r', encoding='utf-8') as f:
            return f.read()

    def _extract_title(self, content: str) -> str:
        """
        Extract title from markdown content

        Args:
            content: Markdown content

        Returns:
            Extracted title
        """
        lines = content.split('\n')
        for line in lines:
            if line.strip().startswith('# '):
                return line.strip()[2:]  # Remove '# ' prefix
        return "Untitled"

    def _extract_chapter_from_path(self, file_path: str) -> str:
        """
        Extract chapter identifier from file path

        Args:
            file_path: Path to the content file

        Returns:
            Chapter identifier
        """
        path_parts = Path(file_path).parts
        for part in path_parts:
            if 'chapter' in part.lower():
                return part.replace('.md', '')
        return Path(file_path).stem

    def _extract_references(self, content: str) -> List[str]:
        """
        Extract references from markdown content

        Args:
            content: Markdown content

        Returns:
            List of reference URLs
        """
        references = []
        lines = content.split('\n')

        # Look for markdown links that might be references
        for line in lines:
            # Simple regex to find markdown links [text](url)
            import re
            matches = re.findall(r'\[([^\]]+)\]\(([^)]+)\)', line)
            for text, url in matches:
                if url.startswith(('http://', 'https://')):
                    references.append(url)

        return references

    def _generate_content_id(self, file_path: str) -> str:
        """
        Generate a unique content ID based on file path and content

        Args:
            file_path: Path to the content file

        Returns:
            Generated content ID
        """
        # Create a hash of the file path and modification time
        path_hash = hashlib.md5(file_path.encode()).hexdigest()[:8]
        mtime = str(os.path.getmtime(file_path)) if os.path.exists(file_path) else str(datetime.now().timestamp())
        time_hash = hashlib.md5(mtime.encode()).hexdigest()[:8]

        return f"vla_{path_hash}_{time_hash}"

    async def _chunk_and_store_content(self, book_content: BookContent) -> List[DocumentChunk]:
        """
        Chunk content and store in vector database

        Args:
            book_content: BookContent object to chunk and store

        Returns:
            List of created DocumentChunk objects
        """
        # Split content into chunks
        chunks = self._create_chunks(book_content)

        stored_chunks = []
        for chunk in chunks:
            try:
                # Generate embedding for the chunk
                embedding = await self._generate_embedding(chunk.text)
                chunk.embedding = embedding

                # Store in vector database
                await self.vector_service.store_chunk(chunk)

                stored_chunks.append(chunk)
            except Exception as e:
                logger.error(f"Error storing chunk {chunk.id}: {str(e)}")

        return stored_chunks

    def _create_chunks(self, book_content: BookContent, max_chunk_size: int = 1000) -> List[DocumentChunk]:
        """
        Create chunks from book content

        Args:
            book_content: BookContent object to chunk
            max_chunk_size: Maximum size of each chunk in characters

        Returns:
            List of DocumentChunk objects
        """
        chunks = []
        content = book_content.body
        content_id = book_content.id

        # Split content into chunks
        paragraphs = content.split('\n\n')
        current_chunk = ""
        chunk_index = 0

        for paragraph in paragraphs:
            # If adding this paragraph would exceed the chunk size
            if len(current_chunk) + len(paragraph) > max_chunk_size and current_chunk:
                # Save the current chunk
                chunk = DocumentChunk(
                    id=f"{content_id}_chunk_{chunk_index}",
                    content_id=content_id,
                    text=current_chunk.strip(),
                    chunk_index=chunk_index,
                    metadata={
                        "module": book_content.module,
                        "chapter": book_content.chapter,
                        "title": book_content.title
                    }
                )
                chunks.append(chunk)

                # Start a new chunk with the current paragraph
                current_chunk = paragraph + "\n\n"
                chunk_index += 1
            else:
                # Add the paragraph to the current chunk
                current_chunk += paragraph + "\n\n"

        # Don't forget the last chunk
        if current_chunk.strip():
            chunk = DocumentChunk(
                id=f"{content_id}_chunk_{chunk_index}",
                content_id=content_id,
                text=current_chunk.strip(),
                chunk_index=chunk_index,
                metadata={
                    "module": book_content.module,
                    "chapter": book_content.chapter,
                    "title": book_content.title
                }
            )
            chunks.append(chunk)

        return chunks

    async def _generate_embedding(self, text: str) -> List[float]:
        """
        Generate embedding for text (mock implementation)

        Args:
            text: Text to generate embedding for

        Returns:
            Embedding vector as list of floats
        """
        # In a real implementation, this would call an embedding API
        # For now, return a mock embedding
        # OpenAI embeddings are typically 1536 dimensions
        import random
        return [random.random() * 2 - 1 for _ in range(1536)]  # Random values between -1 and 1

    async def validate_vla_content(self, content: str) -> Dict[str, Any]:
        """
        Validate VLA module content against technical accuracy standards

        Args:
            content: Content to validate

        Returns:
            Dictionary with validation results
        """
        issues = []

        # Check for common VLA terminology
        required_terms = ["VLA", "Vision-Language-Action", "robot", "simulation"]
        missing_terms = [term for term in required_terms if term.lower() not in content.lower()]
        if missing_terms:
            issues.append(f"Missing key VLA terms: {missing_terms}")

        # Check for proper structure
        lines = content.split('\n')
        has_headers = any(line.strip().startswith('# ') for line in lines)
        if not has_headers:
            issues.append("Content should have proper markdown headers")

        # Check for code examples (if applicable)
        has_code_blocks = '```' in content
        if not has_code_blocks and 'example' in content.lower():
            issues.append("Content mentions examples but has no code blocks")

        # Check for technical depth
        if len(content) < 500:
            issues.append("Content appears to be too brief for a comprehensive VLA module")

        return {
            "valid": len(issues) == 0,
            "issues": issues,
            "content_length": len(content),
            "confidence": 0.8 if len(issues) == 0 else max(0.1, 1.0 - len(issues) * 0.1)
        }

    async def update_content_in_store(self, content_id: str, new_content: BookContent) -> bool:
        """
        Update existing content in the vector store

        Args:
            content_id: ID of content to update
            new_content: New BookContent object

        Returns:
            True if update successful, False otherwise
        """
        try:
            # Remove old chunks
            await self.vector_service.remove_content_chunks(content_id)

            # Add new chunks
            new_chunks = await self._chunk_and_store_content(new_content)

            logger.info(f"Updated content {content_id} with {len(new_chunks)} new chunks")
            return True

        except Exception as e:
            logger.error(f"Error updating content {content_id}: {str(e)}")
            return False

    async def process_vla_module(self, force_reprocess: bool = False) -> Dict[str, Any]:
        """
        Process the entire VLA module

        Args:
            force_reprocess: Whether to reprocess content even if it already exists

        Returns:
            Dictionary with processing results
        """
        logger.info("Starting VLA module processing")

        result = await self.process_vla_content()

        logger.info(f"VLA module processing completed: {result['processed_count']} documents processed")

        return result


# Singleton instance
vla_document_processor = VLADocumentProcessor()