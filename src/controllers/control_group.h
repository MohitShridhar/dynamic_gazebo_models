// Copyright (c) 2014 Mohit Shridhar, David Lee

// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:

// The above copyright notice and this permission notice shall be included in all
// copies or substantial portions of the Software.

// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
// SOFTWARE.

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