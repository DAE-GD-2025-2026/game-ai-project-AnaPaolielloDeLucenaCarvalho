#pragma once
#include <concepts>
#include <any>
#include <vector>

namespace GameAI
{
	template <typename NodeType>
	concept is_node_type = requires (NodeType Node)
	{
		{Node.SetId(std::declval<int>())};
		{Node.GetId()} -> std::same_as<int>;
	};
	
	template <typename NodeType>
	concept is_drawable_node = requires (NodeType Node)
	{
		requires is_node_type<NodeType>;
		requires std::constructible_from<NodeType, FVector2D>;
		{Node.GetPosition()} -> std::same_as<FVector2D const &>;
		{Node.SetPosition(std::declval<FVector2D const &>())};
	};

	template <typename ConnectionType>
	concept is_connection_type = requires (ConnectionType Connection)
	{
		requires std::constructible_from<ConnectionType, int, int>;
		requires std::equality_comparable<ConnectionType>;
		
		{Connection.GetInverseCopy()} -> std::same_as<ConnectionType>;
		{Connection.GetFromId()} -> std::same_as<int>;
		{Connection.GetToId()} -> std::same_as<int>;
		{Connection.GetWeight()} -> std::same_as<float>;
		{Connection.SetWeight(std::declval<float>())};
	};
}
