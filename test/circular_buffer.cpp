#include <vector>

#include "catch.hpp"

#include "../circular_buffer.hpp"

using namespace dsp;
using namespace std;

TEST_CASE("CircularBuffer")
{
	SECTION("Construct empty initialization")
	{
		CircularBuffer<int> buffer(5);

		CHECK(buffer.size() == 5);
		for (auto i = 0; i < 5; ++i)
			CHECK(buffer[i] == 0);

		SECTION("size 0")
		{
			CircularBuffer<int> buffer(0);
			CHECK(buffer.size() == 0);
		}
	}

	SECTION("Construct with initializer list")
	{
		CircularBuffer<int> buffer = { 4, 4, 4, 4 };

		CHECK(buffer.size() == 4);
		for (auto i = 0; i < 4; ++i)
			CHECK(buffer[i] == 4);
	}

	SECTION("Construct with iterators")
	{
		vector<int> vec = { 1, 1, 1, 1, 1, 1 };
		CircularBuffer<int> buffer(vec.begin(), vec.end());

		CHECK(buffer.size() == 6);
		for (auto i = 0; i < 6; ++i)
			CHECK(buffer[i] == 1);
	}

	SECTION("Resizing")
	{
		CircularBuffer<int> buffer = { 1, 2, 3, 4 };

		SECTION("resize_front")
		{
			buffer.resize_front(5);
			CHECK(buffer[0] == 0);
			CHECK(buffer[4] == 4);
		}

		SECTION("resize_back")
		{
			buffer.resize_back(5);
			CHECK(buffer[0] == 1);
			CHECK(buffer[4] == 0);
		}
	}

	SECTION("write()")
	{
		CircularBuffer<int> buffer(3);	
		buffer.write(8);
		buffer.write(9);

		CHECK(buffer.size() == 3);
		CHECK(buffer[buffer.size() - 2] == 8);
		CHECK(buffer[buffer.size() - 1] == 9);

		buffer.write(1);
		buffer.write(13);
		buffer.write(-8);

		CHECK(buffer[buffer.size() - 3] == 1);
		CHECK(buffer[buffer.size() - 2] == 13);
		CHECK(buffer[buffer.size() - 1] == -8);
	}

	SECTION("Subscript")
	{
		CircularBuffer<int> buffer(4);

		for (auto i = 0; i < 4; ++i)
		{
			CHECK_NOTHROW(buffer[0]);
			buffer[0] = i * 2;
			CHECK(buffer[0] == i + i);
		}

		SECTION("Out of Range")
		{
			CHECK_THROWS_AS(buffer[4], std::out_of_range);
			CHECK_THROWS_AS(buffer[15], std::out_of_range);
			CHECK_THROWS_AS(buffer[-1], std::out_of_range);
		}
	}

	SECTION("Iterators")
	{
		CircularBuffer<int> buffer = { 0, 1, 2, 3, 4 };

		SECTION("regular")
		{
			int i = 0;
			for (auto it = buffer.begin(); it != buffer.end(); ++it)
				CHECK(*it == i++);
		}

		SECTION("regular const")
		{
			int i = 0;
			for (auto it = buffer.cbegin(); it != buffer.cend(); ++it)
				CHECK(*it == i++);
		}

		SECTION("reverse")
		{
			int i = 4;
			for (auto it = buffer.rbegin(); it != buffer.rend(); ++it)
				CHECK(*it == i--);
		}

		SECTION("reverse const")
		{
			int i = 4;
			for (auto it = buffer.crbegin(); it != buffer.crend(); ++it)
				CHECK(*it == i--);
		}

		SECTION("size 0")
		{
			CircularBuffer<int> buffer(0);
			CHECK(buffer.begin() == buffer.end());
			CHECK(buffer.cbegin() == buffer.cend());
			CHECK(buffer.rbegin() == buffer.rend());
			CHECK(buffer.crbegin() == buffer.crend());
		}
	}
}