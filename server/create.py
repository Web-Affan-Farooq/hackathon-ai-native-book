from qdrant_client import QdrantClient
from qdrant_client.models import VectorParams, Distance
import os
from dotenv import load_dotenv

load_dotenv()

client = QdrantClient(
    url=os.getenv("QDRANT_ENDPOINT"),
    api_key=os.getenv("QDRANT_API_KEY")
)

client.recreate_collection(
    collection_name="physical-ai-humanoid-robotics",
    vectors_config=VectorParams(
        size=1024,              # ✅ MUST match Cohere
        distance=Distance.COSINE
    )
)

print("✅ Collection recreated with vector size 1024")
