// SPDX-License-Identifier: BUSL-1.1
#if WITH_LOW_LEVEL_TESTS

#include "catch_amalgamated.hpp"
#include "OGSimulation/SimulationComposite.h"

// ---------------------------------------------------------------------------
// Test: mutable forEach visits every element exactly once, in tuple order.
// ---------------------------------------------------------------------------
TEST_CASE("DAttack.SimulationComposite.ForEachMutableVisitsAllInOrder", "[DAttack][SimulationComposite]")
{
    struct A { int value = 0; };
    struct B { int value = 0; };
    struct C { int value = 0; };

    SimulationComposite<A, B, C> composite;
    int visitOrder[3] = {-1, -1, -1};
    int visitIndex = 0;

    composite.forEach([&](auto& elem) {
        elem.value = visitIndex + 10;
        visitOrder[visitIndex++] = elem.value;
    });

    REQUIRE(visitIndex == 3);
    REQUIRE(composite.get<A>().value == 10);
    REQUIRE(composite.get<B>().value == 11);
    REQUIRE(composite.get<C>().value == 12);
}

// Callable that is SFINAE-rejected if passed a non-const ref: only binds to const T&.
// Declared at file scope because MSVC does not allow template members on local classes.
struct FConstOnlyVisitor
{
    int sum = 0;
    template <typename T>
    void operator()(const T& elem) { sum += elem.value; }
};

struct FConstTestX { int value = 7; };
struct FConstTestY { int value = 13; };

// ---------------------------------------------------------------------------
// Test: const forEach accepts only const-ref callables, visits all elements.
// ---------------------------------------------------------------------------
TEST_CASE("DAttack.SimulationComposite.ForEachConstVisitsWithConstRef", "[DAttack][SimulationComposite]")
{
    SimulationComposite<FConstTestX, FConstTestY> composite;
    const SimulationComposite<FConstTestX, FConstTestY>& constRef = composite;

    FConstOnlyVisitor visitor;
    constRef.forEach(visitor);

    REQUIRE(visitor.sum == 7 + 13);
}

// ---------------------------------------------------------------------------
// Test: forEach on an empty composite is a no-op.
// ---------------------------------------------------------------------------
TEST_CASE("DAttack.SimulationComposite.ForEachEmptyIsNoOp", "[DAttack][SimulationComposite]")
{
    SimulationComposite<> empty;
    int callCount = 0;
    empty.forEach([&](auto&) { ++callCount; });

    REQUIRE(callCount == 0);

    const SimulationComposite<>& constEmpty = empty;
    constEmpty.forEach([&](const auto&) { ++callCount; });

    REQUIRE(callCount == 0);
}

#endif // WITH_LOW_LEVEL_TESTS
