#include "Graph.h"
#include <ranges>

namespace GameAI
{
#pragma region Nodes
    Node::Node(FVector2D const& Position)
        : Position(Position)
    {
    }

    void Node::SetId(int id)
    {
        Id = id;
    }
    int Node::GetId() const
    {
        return Id;
    }

    FVector2D const& Node::GetPosition() const
    {
        return Position;
    }
    void Node::SetPosition(FVector2D const& NewPos)
    {
        Position = NewPos;
    }

    bool Node::operator==(const Node& Other) const
    {
        return Id == Other.Id;
    }
    bool Node::operator==(const Node* OtherPtr) const
    {
        return Id == OtherPtr->Id;
    }
    
#pragma endregion Nodes

#pragma region Connections
    Connection::Connection(int FromId, int ToId)
        : FromId(FromId)
        , ToId(ToId)
    {
    }

    int Connection::GetFromId() const
    {
        return FromId;
    }

    int Connection::GetToId() const
    {
        return ToId;
    }

    float Connection::GetWeight() const
    {
        return Weight;
    }

    void Connection::SetWeight(float NewWeight)
    {
        Weight = NewWeight;
    }

    Connection Connection::GetInverseCopy() const
    {
        return Connection{ToId, FromId};
    }

    bool Connection::operator==(const Connection& Other) const
    {
        return FromId == Other.GetFromId() && ToId == Other.GetToId();
    }
#pragma endregion Connections

#pragma region Graph
    Graph::Graph(bool isDirectional)
        : bIsDirectional{isDirectional}
    {
    }

    Graph::Graph(Graph const & Other)
        : bIsDirectional{Other.bIsDirectional}
    {
        Nodes.reserve(Other.Nodes.size());
        for (std::unique_ptr<Node> const & OtherNode : Other.Nodes)
        {
            Nodes.push_back(std::make_unique<Node>(*OtherNode.get()));
        }
        
        Connections.reserve(Other.Connections.size());
        for (std::unique_ptr<Connection> const & OtherConnection : Other.Connections)
        {
            Connections.push_back(std::make_unique<Connection>(*OtherConnection.get()));
        }
    }

    // --- Nodes ------------------------------------------------------------
    std::vector<std::unique_ptr<Node>> const& Graph::GetNodes() const
    {
        return Nodes;
    }

    std::vector<std::unique_ptr<Node>>& Graph::GetNodes()
    {
        return Nodes;
    }

    std::vector<Node const *> Graph::GetActiveNodes() const
    {
        std::vector<Node const *> ActiveNodes{};
        for (auto const & Node : Nodes)
        {
            if (Node->GetId() >= 0)
            {
                ActiveNodes.push_back(Node.get());
            }
        }
        return ActiveNodes;
    }

    std::vector<Node*> Graph::GetActiveNodes()
    {
        std::vector<Node*> ActiveNodes{};
        for (auto& Node : Nodes)
        {
            if (Node->GetId() >= 0)
            {
                ActiveNodes.push_back(Node.get());
            }
        }
        return ActiveNodes;
    }

    int Graph::GetNodeCount() const
    {
        return std::count_if(Nodes.begin(), Nodes.end(),
            [](auto const& Element) { return Element->GetId() >= 0; });
    }

    std::unique_ptr<Node> const& Graph::GetNode(int NodeId) const
    {
        return Nodes[NodeId];
    }

    std::unique_ptr<Node>& Graph::GetNode(int NodeId)
    {
        return Nodes[NodeId];
    }

    int Graph::AddNode(std::unique_ptr<Node> NewNode)
    {
        // reuse invalidated node slots if possible
        if (auto InvalidIndex = GetFirstInvalidNodeIdx(); InvalidIndex.has_value())
        {
            Nodes[InvalidIndex.value()].reset();
            NewNode->SetId(InvalidIndex.value());
            Nodes[InvalidIndex.value()] = std::move(NewNode);
            return InvalidIndex.value();
        }

        NewNode->SetId(static_cast<int>(Nodes.size()));
        Nodes.push_back(std::move(NewNode));
        return Nodes.back()->GetId();
    }

    bool Graph::RemoveNode(int NodeToRemoveId)
    {
        if (NodeToRemoveId < 0 || 
            NodeToRemoveId >= static_cast<int>(Nodes.size()) ||
            Nodes[NodeToRemoveId]->GetId() != NodeToRemoveId)
        {
            UE_LOG(LogTemp, Warning, TEXT("Attempted to remove a node not in this graph!"));
            return false;
        }

        // Remove all connections that involve this node
        std::erase_if(Connections,
            [NodeToRemoveId](auto const& Connection)
            {
                return Connection->GetFromId() == NodeToRemoveId || Connection->GetToId() == NodeToRemoveId;
            });

        // Mark node as invalid (keep it in the vector to preserve indices)
        Nodes[NodeToRemoveId]->SetId(Graphs::InvalidNodeId);
        return true;
    }

    // --- Connections ------------------------------------------------------
    std::vector<std::unique_ptr<Connection>> const& Graph::GetConnections() const
    {
        return Connections;
    }

    std::vector<std::unique_ptr<Connection>>& Graph::GetConnections()
    {
        return Connections;
    }

    Connection* Graph::FindConnection(int FromId, int ToId)
    {
        auto it = std::find_if(Connections.begin(), Connections.end(),
            [=](auto const& Element)
            {
                return Element->GetFromId() == FromId && Element->GetToId() == ToId;
            });
        return it != Connections.end() ? it->get() : nullptr;
    }

    std::vector<Connection*> Graph::FindConnectionsFrom(int NodeId) const
    {
        std::vector<Connection*> Result{};
        for (auto& Connection : Connections)
        {
            if (Connection->GetFromId() == NodeId)
                Result.push_back(Connection.get());
        }
        return Result;
    }

    std::vector<Connection*> Graph::FindConnectionsTo(int NodeId) const
    {
        std::vector<Connection*> Result{};
        for (auto& Connection : Connections)
        {
            if (Connection->GetToId() == NodeId)
                Result.push_back(Connection.get());
        }
        return Result;
    }

    void Graph::AddConnection(std::unique_ptr<Connection> NewConnection)
    {
        // Get an inverse copy for later
        auto InverseNew = NewConnection->GetInverseCopy();
        
        // Check if the connection already exists
        auto Found = std::find_if(Connections.begin(), Connections.end(),
            [&](auto const& Existing)
            {
                return Existing->GetFromId() == NewConnection->GetFromId() &&
                       Existing->GetToId() == NewConnection->GetToId();
            });

        if (Found != Connections.end())
        {
            UE_LOG(LogTemp, Warning, TEXT("Attempted to add a connection already in the graph!"));
            return;
        }

        // Add the new connection
        Connections.push_back(std::move(NewConnection));

        if (!bIsDirectional)
        {
            // Also add the inverse connection
            Connections.push_back(std::make_unique<Connection>(InverseNew));
        }
    }

    void Graph::AddConnection(int FromNodeId, int ToNodeId)
    {
        AddConnection(std::make_unique<Connection>(FromNodeId, ToNodeId));
    }

    bool Graph::RemoveConnection(Connection const* ConnectionToRemove)
    {
        // Stored for later use
        auto InverseConnection = ConnectionToRemove->GetInverseCopy();
			
        int AmountRemoved{0};
        AmountRemoved += std::erase_if(Connections,
            [&](std::unique_ptr<Connection> const & Element){return *Element.get() == *ConnectionToRemove;});
        if (!bIsDirectional)
        {
            // Remove the inverse
            AmountRemoved += std::erase_if(Connections,
                [&](std::unique_ptr<Connection> const & Element){return *Element.get() == InverseConnection;});
        }
			
        return AmountRemoved > 0;
    }

    bool Graph::RemoveConnection(int FromNodeId, int ToNodeId)
    {
        
        if (auto FindResult = FindConnection(FromNodeId, ToNodeId))
        {
            return RemoveConnection(FindResult);
        }
        UE_LOG(LogTemp, Warning, TEXT("Attempted to remove non-existant connection from %d to %d"),
            FromNodeId, ToNodeId);
        return false;
    }

    bool Graph::GetIsDirectional() const
    {
        return bIsDirectional;
    }

    Graph Graph::Clone() const
    {
        return Graph{*this};
    }

    void Graph::SetConnectionCostsToDistances()
    {
        for (auto& Connection : Connections)
        {
            FVector2D BetweenNodes{GetNode(Connection->GetFromId())->GetPosition() - GetNode(Connection->GetToId())->GetPosition()};
            Connection->SetWeight(BetweenNodes.Length());
        }
    }

    std::optional<int> Graph::GetFirstInvalidNodeIdx() const
    {
        for (int Index{0}; Index < Nodes.size(); ++Index)
        {
            if (Nodes[Index]->GetId() == Graphs::InvalidNodeId)
            {
                return Index;
            }
        }
        
        return std::nullopt;
    }
#pragma endregion Graph
}