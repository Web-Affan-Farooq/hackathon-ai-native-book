from embedding import Embedding

emb = Embedding()
emb.create_rag()

# import os 
# from dotenv import load_dotenv
# import cohere
# from qdrant_client import QdrantClient

# load_dotenv()
# # Initialize Cohere client
# cohere_client = cohere.Client("COHERE_API_KEY")

# # Connect to Qdrant
# Qclient = QdrantClient(
#             url=os.getenv("QDRANT_ENDPOINT"),
#             api_key=os.getenv("QDRANT_API_KEY")
#         )

# def get_embedding(text):
#     """Get embedding vector from Cohere Embed v3"""
#     response = cohere_client.embed(
#         model="embed-english-v3.0",
#         input_type="search_query",  # Use search_query for queries
#         texts=[text],
#     )
#     return response.embeddings[0]  # Return the first embedding

# def retrieve(query):
#     embedding = get_embedding(query)
#     result = Qclient.query_points(
#         collection_name="humanoid_ai_book_two",
#         query=embedding,
#         limit=5
#     )
#     return [point.payload["text"] for point in result.points]

# # Test
# print(retrieve("What data do you have?"))