#include "removable.hpp"

Removable::~Removable()
{
}

void Removable::remove()
{
	removed = true;
}

bool Removable::ifRemoved()
{
	return removed;
}
