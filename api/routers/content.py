from fastapi import APIRouter, HTTPException
from typing import Dict, Any, List
from ..services.vector_store import vector_db
from ..services.rag_service import rag_service
from ..utils.logging import logger
from ..utils.errors import RAGException

router = APIRouter(prefix="/content", tags=["content"])


@router.post("/search")
async def search_content(query: Dict[str, Any]):
    """Search for relevant content in the book based on query."""
    try:
        search_query = query.get("query", "")
        limit = min(query.get("limit", 5), 20)  # Cap the limit at 20
        filters = query.get("filters", {})

        logger.info(
            f"Content search request",
            extra={
                "query": search_query[:100] + "..." if len(search_query) > 100 else search_query,
                "limit": limit,
                "filters": filters
            }
        )

        # Validate the search query
        if not search_query or not search_query.strip():
            raise HTTPException(status_code=400, detail="Search query cannot be empty")

        # In a real implementation, we would:
        # 1. Create an embedding for the search query
        # 2. Search the vector database for similar content
        # 3. Apply any filters if specified
        # 4. Return the results with relevance scores

        # For now, use the RAG service to get relevant content
        # This is a simplified approach - in a real system we would have dedicated search functionality
        results = await rag_service.get_relevant_content(search_query, limit=limit)

        # Format results to match the expected schema
        formatted_results = []
        for result in results:
            formatted_results.append({
                "contentId": result.get("content_id", ""),
                "title": result.get("metadata", {}).get("source_title", "Untitled"),
                "excerpt": result.get("text", "")[:200] + "..." if len(result.get("text", "")) > 200 else result.get("text", ""),
                "url": f"/docs/{result.get('metadata', {}).get('source_slug', 'unknown')}",
                "relevance": result.get("score", 0.0),
                "tags": result.get("metadata", {}).get("tags", [])
            })

        logger.info(
            f"Content search completed",
            extra={
                "result_count": len(formatted_results),
                "query": search_query[:50] + "..."
            }
        )

        return {
            "results": formatted_results,
            "query": search_query,
            "total": len(formatted_results)
        }

    except HTTPException:
        # Re-raise HTTP exceptions
        raise
    except Exception as e:
        logger.error(
            f"Error in content search",
            extra={
                "query": search_query if 'search_query' in locals() else "unknown",
                "error": str(e)
            },
            exc_info=True
        )
        raise HTTPException(status_code=500, detail=f"Error performing content search: {str(e)}")