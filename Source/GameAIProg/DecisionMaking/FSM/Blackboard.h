#pragma once
#include <unordered_map>
#include <string>
#include <any>

namespace GameAI::FSM
{
	class Blackboard
	{
	public:
		template<typename T>
		T GetData(const std::string& Key)
		{
			if (Data.find(Key) != Data.end())
			{
				return std::any_cast<T>(Data[Key]);
			}
			return T(); 
		}

		template<typename T>
		void SetData(const std::string& Key, T Value)
		{
			Data[Key] = Value;
		}

	private:
		std::unordered_map<std::string, std::any> Data;
	};
}