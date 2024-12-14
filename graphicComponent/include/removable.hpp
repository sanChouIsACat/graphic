#pragma once
class Removable
{
public:
	~Removable();
	void remove();
	bool ifRemoved();

protected:
	bool removed;
};