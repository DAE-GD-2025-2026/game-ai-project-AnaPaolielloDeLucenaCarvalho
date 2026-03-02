#pragma once

#include <memory>
#include <vector>
#include <algorithm>
#include <ranges>

namespace GameAI
{
    namespace Graphs
    {
        static int constexpr InvalidNodeId = -1;
    }

    // Forward declarations
    class Graph;

#pragma region NodeType(s)
    class Node
    {
    public:
        explicit Node(FVector2D const& Position);
        void SetId(int id);
        int GetId() const;

        FVector2D const& GetPosition() const;
        void SetPosition(FVector2D const& NewPos);
        
        bool operator==(const Node& Other) const;
        bool operator==(const Node* OtherPtr) const;

    private:
        FVector2D Position;
        int Id = Graphs::InvalidNodeId;
    };
#pragma endregion NodeType(s)

#pragma region ConnectionType(s)
    class Connection
    {
    public:
        Connection(int FromId, int ToId);

        int GetFromId() const;
        int GetToId() const;
        float GetWeight() const;
        void SetWeight(float NewWeight);
        
        Connection GetInverseCopy() const;
        bool operator==(const Connection& Other) const;
    
    private:
        int FromId;
        int ToId;
        float Weight{};
    };
#pragma endregion ConnectionType(s)


    class Graph
    {
    public:
        explicit Graph(bool isDirectional = false);
        explicit Graph(Graph const & Other);

        // --- Nodes --------------------------------------------------------
        std::vector<std::unique_ptr<Node>> const& GetNodes() const;
        std::vector<std::unique_ptr<Node>>& GetNodes();

        std::vector<Node const *>  GetActiveNodes() const;
        std::vector<Node*> GetActiveNodes();
        int GetNodeCount() const;

        std::unique_ptr<Node> const& GetNode(int NodeId) const;
        std::unique_ptr<Node>& GetNode(int NodeId);

        int AddNode(std::unique_ptr<Node> NewNode);   // takes ownership
        bool RemoveNode(int NodeToRemoveId);

        // --- Connections --------------------------------------------------
        std::vector<std::unique_ptr<Connection>> const& GetConnections() const;
        std::vector<std::unique_ptr<Connection>>& GetConnections();

        Connection* FindConnection(int FromId, int ToId);         
        std::vector<Connection*> FindConnectionsFrom(int NodeId) const;           
        std::vector<Connection*> FindConnectionsTo(int NodeId) const;

        void AddConnection(std::unique_ptr<Connection> NewConnection);
        void AddConnection(int FromNodeId, int ToNodeId);

        bool RemoveConnection(Connection const* ConnectionToRemove);
        bool RemoveConnection(int FromNodeId, int ToNodeId);

        bool GetIsDirectional() const;
        Graph Clone() const;
        
        // Helper
        void SetConnectionCostsToDistances();

    private:
        std::optional<int> GetFirstInvalidNodeIdx() const;
        
        bool const bIsDirectional;
        std::vector<std::unique_ptr<Node>> Nodes;
        std::vector<std::unique_ptr<Connection>> Connections;
    };
}
