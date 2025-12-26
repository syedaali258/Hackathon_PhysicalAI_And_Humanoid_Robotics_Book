import os
import asyncio
from typing import List
from api.models.document import BookContent, ContentType
from api.services.embedding_service import embedding_service
from api.utils.logging import logger
import uuid


class ContentLoader:
    """Utility class to load book content from markdown files and add to the RAG system."""

    def __init__(self, content_dir: str = "book/docs"):
        self.content_dir = content_dir

    async def load_content_from_file(self, file_path: str) -> BookContent:
        """Load content from a markdown file and convert to BookContent model."""
        with open(file_path, 'r', encoding='utf-8') as f:
            content_text = f.read()

        # Extract title from the file content (first heading)
        title = self._extract_title(content_text)

        # Generate a slug from the file path
        slug = self._generate_slug(file_path)

        # Create a content ID
        content_id = str(uuid.uuid5(uuid.NAMESPACE_URL, file_path))

        # Determine content type based on the path
        content_type = self._determine_content_type(file_path)

        # Extract metadata from frontmatter if present
        content_text, metadata = self._extract_frontmatter(content_text)

        # Create BookContent object
        book_content = BookContent(
            id=content_id,
            type=content_type,
            title=title,
            slug=slug,
            content=content_text,
            metadata=metadata,
            hierarchy=self._extract_hierarchy(file_path),
            references=[]  # Could extract references from content
        )

        return book_content

    def _extract_title(self, content: str) -> str:
        """Extract title from the content (first H1 heading)."""
        lines = content.split('\n')
        for line in lines:
            if line.strip().startswith('# '):
                return line.strip()[2:]  # Remove '# ' prefix
        return "Untitled"

    def _generate_slug(self, file_path: str) -> str:
        """Generate a URL-friendly slug from the file path."""
        # Get the relative path from content_dir
        rel_path = os.path.relpath(file_path, self.content_dir)
        # Replace slashes with hyphens and remove extension
        slug = rel_path.replace(os.sep, '-').replace('/', '-').replace('\\', '-')
        slug = os.path.splitext(slug)[0]  # Remove extension
        return slug.lower()

    def _determine_content_type(self, file_path: str) -> ContentType:
        """Determine content type based on the file path."""
        path_parts = file_path.split(os.sep)
        if 'module' in file_path.lower():
            if 'chapter' in file_path.lower():
                return ContentType.chapter
            else:
                return ContentType.module
        elif 'intro' in file_path.lower():
            return ContentType.section
        else:
            return ContentType.section

    def _extract_frontmatter(self, content: str):
        """Extract frontmatter from content if present."""
        if content.startswith('---'):
            parts = content.split('---', 2)
            if len(parts) >= 3:
                # Parse frontmatter
                frontmatter = parts[1]
                content_body = parts[2]

                # Simple frontmatter parsing (in a real app, use a YAML parser)
                metadata = {}
                for line in frontmatter.split('\n'):
                    if ':' in line:
                        key, value = line.split(':', 1)
                        key = key.strip()
                        value = value.strip().strip('"\'')
                        metadata[key] = value

                return content_body, metadata

        return content, {}

    def _extract_hierarchy(self, file_path: str) -> dict:
        """Extract hierarchy information from the file path."""
        rel_path = os.path.relpath(file_path, self.content_dir)
        path_parts = rel_path.split(os.sep)

        hierarchy = {}
        for i, part in enumerate(path_parts):
            if 'module' in part.lower():
                hierarchy['moduleNumber'] = self._extract_number(part)
            elif 'chapter' in part.lower():
                hierarchy['chapterNumber'] = self._extract_number(part)

        return hierarchy

    def _extract_number(self, s: str) -> int:
        """Extract number from a string like 'module-1-ros2'."""
        import re
        numbers = re.findall(r'\d+', s)
        return int(numbers[0]) if numbers else 0

    async def load_all_content(self) -> List[BookContent]:
        """Load all markdown content from the content directory."""
        contents = []

        for root, dirs, files in os.walk(self.content_dir):
            for file in files:
                if file.endswith('.md') or file.endswith('.mdx'):
                    file_path = os.path.join(root, file)
                    try:
                        content = await self.load_content_from_file(file_path)
                        contents.append(content)
                        logger.info(f"Loaded content: {content.title} from {file_path}")
                    except Exception as e:
                        logger.error(f"Error loading content from {file_path}: {str(e)}")

        return contents

    async def load_and_process_content(self):
        """Load all content and process it for the RAG system."""
        logger.info("Starting content loading and processing")

        contents = await self.load_all_content()

        processed_count = 0
        for content in contents:
            try:
                await embedding_service.process_content(content)
                processed_count += 1
                logger.info(f"Processed content: {content.title}")
            except Exception as e:
                logger.error(f"Error processing content {content.title}: {str(e)}")

        logger.info(f"Completed processing {processed_count} content items")

        return contents


# Example usage
async def load_sample_content():
    """Load sample content for testing."""
    loader = ContentLoader()
    contents = await loader.load_and_process_content()
    return contents