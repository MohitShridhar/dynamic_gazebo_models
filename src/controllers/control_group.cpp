enum GroupType {DOOR, ELEVATOR, INVALID};

class ControlGroup
{	
	private:

		std::string group_name;
		GroupType type;
		std::vector<uint32_t> active_units;

	public:

		ControlGroup(std::string group_name, GroupType type, std::vector<uint32_t> active_units)
		{
			this->group_name = group_name;
			this->type = type;
			this->active_units = active_units;
		}

		std::string getGroupName()
		{
			return this->group_name;
		}

		void setGroupName(std::string group_name)
		{
			this->group_name = group_name;
		}

		GroupType getType()
		{
			return this->type;
		}

		void setGroupType(GroupType type)
		{
			this->type = type;
		}

		std::vector<uint32_t> getActiveUnits()
		{
			return this->active_units;
		}

		void setActiveUnits(std::vector<uint32_t> active_units)
		{
			this->active_units = active_units;
		}
};