#pragma once
#include <vector>


class TriPolygon
{
public:
	
	struct Edge
	{
		std::array<int, 2> EdgeIndices;
		
		bool operator==(const Edge& Other) const;
		
		FVector GetP1(TriPolygon const& Poly) const { return Poly.Vertices[EdgeIndices[0]]; }
		FVector GetP2(TriPolygon const& Poly) const { return Poly.Vertices[EdgeIndices[1]]; }
	};
	
	struct Triangle
	{
		std::array<int, 3> VertexIndices;
		
		FVector GetVertex0(TriPolygon const& Poly) const { return Poly.Vertices[VertexIndices[0]]; }
		FVector GetVertex1(TriPolygon const& Poly) const { return Poly.Vertices[VertexIndices[1]]; }
		FVector GetVertex2(TriPolygon const& Poly) const { return Poly.Vertices[VertexIndices[2]]; }
		
		std::array<FVector, 3> GetVertices(TriPolygon const& Poly) const;
		std::vector<int> GetNeighbors(TriPolygon const& Poly) const;
		std::array<Edge, 3> GetEdges() const; // helper
		
		bool operator==(const Triangle& Other) const;
		bool Equals(const TArray<FVector>& VertexData, TriPolygon const & Poly) const;
		bool HasEdge(Edge const & Edge) const;
		
		void DebugDraw(const UWorld* World, TriPolygon const & Poly, FColor const & Color) const;
	};
	
	TriPolygon() = default;
	
	int AddTriangle(TArray<FVector> const & TriangleData);
	
	std::vector<FVector> const& GetVertices() const { return Vertices; }
	std::vector<Edge> const& GetEdges() const { return Edges; }
	std::vector<Triangle> const& GetTriangles() const { return Triangles; }
	Triangle const& GetTriangle(int TriIdx) const { return Triangles[TriIdx]; }
	
	void DrawDebug(UWorld const * World, FColor const & Color) const;
	
	// Queries
	std::vector<int> GetTriangleNeighbors(int InTriangleIndex) const;
	std::vector<int> GetTriangleNeighbors(Triangle const& InTriangle) const;
	
	std::optional<int> FindTriangleIndex(TArray<FVector> const& TriangleData) const;
	std::optional<int> FindVertexIndex(FVector const& Vertex) const;
	std::optional<int> FindEdgeIndex(Edge const& Edge) const;

	Triangle const* GetClosestTriangleToPosition(FVector2D const& DesiredPosition, FVector2D& OutPosition) const;
	Triangle const* GetTriangleAtPosition(FVector2D const& Position, bool OnLineAllowed) const;
	

private:
	int AddVertex(FVector const& Vertex);
	int AddEdge(Edge const & Edge);
	
	std::vector<FVector>  Vertices;
	std::vector<Edge> Edges;
	std::vector<Triangle> Triangles;
};
