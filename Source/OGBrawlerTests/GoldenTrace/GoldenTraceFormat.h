#pragma once
// SPDX-License-Identifier: BUSL-1.1
// docs/GoldenTraceFormat-rationale.md

#include <algorithm>
#include <array>
#include <bit>
#include <charconv>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <limits>
#include <optional>
#include <string>
#include <string_view>
#include <system_error>
#include <vector>

#include "glm/gtc/quaternion.hpp"
#include "glm/vec2.hpp"
#include "glm/vec3.hpp"

#include "OGSimulation/OGAssert.h"
#include "OGSimulation/SimulationFrame.h"

namespace goldenTrace
{

enum class Tag : uint8_t
{
    Position = 0,
    Direction = 1,
    Axial = 2,
    Rotation = 3,
    Scalar = 4,
    Planar = 5,
    Exact = 6,
    Count = 7
};

inline constexpr std::size_t kTagCount = static_cast<std::size_t>(Tag::Count);
inline constexpr char kTagLetters[] = "pdaqs2e";
static_assert(sizeof(kTagLetters) - 1u == kTagCount,
    "goldenTrace::kTagLetters spells one letter per Tag, in Tag order: the .inl files store the letter.");

inline constexpr uint32_t kFormatVersion = 1u;
inline constexpr uint32_t kEnvelopeToleranceFactor = 4u;
inline constexpr uint32_t kUlpFloorFactor = 4u;

constexpr char tagLetter(Tag tag) { return kTagLetters[static_cast<std::size_t>(tag)]; }

constexpr std::optional<Tag> tagOfLetter(char letter)
{
    for (std::size_t index = 0; index < kTagCount; ++index)
        if (kTagLetters[index] == letter)
            return static_cast<Tag>(index);
    return std::nullopt;
}

static_assert(tagOfLetter('2') == Tag::Planar && tagOfLetter('e') == Tag::Exact && !tagOfLetter('x').has_value(),
    "goldenTrace::tagOfLetter must invert tagLetter for every tag and refuse any other letter.");

struct FieldRecord
{
    const char* name;
    char tag;
    uint32_t components;
};

struct GoldenHeader
{
    uint32_t formatVersion;
    uint32_t axisFrame;
    float unitsPerMetre;
    float dtSeconds;
    uint32_t scenarioTicks;
    const char* sourceCommit;
    const char* generator;
};

struct NudgeRecord
{
    uint32_t characterId;
    uint32_t component;
    int32_t direction;
};

struct EnvelopeHeader
{
    uint32_t formatVersion;
    uint32_t runs;
    uint32_t axisFrame;
    float unitsPerMetre;
    const char* sourceCommit;
};

struct FieldSpec
{
    std::string name;
    Tag tag = Tag::Exact;
    uint32_t components = 0;
    uint32_t offset = 0;
};

struct Trace
{
    std::vector<FieldSpec> fields;
    std::vector<uint32_t> ticks;
    std::vector<double> values;
    uint32_t valuesPerSample = 0;

    const double* sample(std::size_t index) const { return values.data() + index * valuesPerSample; }

    std::optional<std::size_t> fieldIndex(std::string_view name) const
    {
        for (std::size_t index = 0; index < fields.size(); ++index)
            if (fields[index].name == name)
                return index;
        return std::nullopt;
    }
};

struct Nudge
{
    uint32_t characterId = 0;
    uint32_t component = 0;
    int32_t direction = 0;
};

class Recorder
{
public:
    using Frame = simFrame;

    explicit Recorder(Trace& trace) : m_trace(trace) {}

    void beginSample(uint32_t tick)
    {
        m_trace.ticks.push_back(tick);
        m_cursor = 0;
        m_firstSample = m_trace.ticks.size() == 1u;
    }

    void endSample()
    {
        if (m_firstSample)
            m_trace.valuesPerSample = static_cast<uint32_t>(m_trace.values.size());
        OG_CHECK(m_cursor == m_trace.fields.size()
                && m_trace.values.size() == m_trace.ticks.size() * std::size_t(m_trace.valuesPerSample),
            "goldenTrace::Recorder::endSample: every sample must record the same fields, in the same order, as the first one");
    }

    void position(const std::string& name, const glm::vec3& value) { push(name, Tag::Position, Frame::toLegacyPosition(value)); }
    void direction(const std::string& name, const glm::vec3& value) { push(name, Tag::Direction, Frame::toLegacyAxes(value)); }
    void axial(const std::string& name, const glm::vec3& value) { push(name, Tag::Axial, Frame::toLegacyAxial(value)); }

    void rotation(const std::string& name, const glm::quat& value)
    {
        const glm::dquat legacy = Frame::toLegacyRotation(value);
        const double components[4] = { legacy.w, legacy.x, legacy.y, legacy.z };
        push(name, Tag::Rotation, components, 4u);
    }

    void scalar(const std::string& name, float value)
    {
        const double component = static_cast<double>(value);
        push(name, Tag::Scalar, &component, 1u);
    }

    void planar(const std::string& name, const glm::vec2& value)
    {
        const glm::dvec2 legacy = Frame::toLegacyPlanar(value);
        const double components[2] = { legacy.x, legacy.y };
        push(name, Tag::Planar, components, 2u);
    }

    void exact(const std::string& name, uint32_t value)
    {
        const double component = static_cast<double>(value);
        push(name, Tag::Exact, &component, 1u);
    }

private:
    void push(const std::string& name, Tag tag, const glm::dvec3& value)
    {
        const double components[3] = { value.x, value.y, value.z };
        push(name, tag, components, 3u);
    }

    void push(const std::string& name, Tag tag, const double* components, uint32_t count)
    {
        if (m_firstSample)
        {
            m_trace.fields.push_back(FieldSpec{ name, tag, count, static_cast<uint32_t>(m_trace.values.size()) });
        }
        else
        {
            OG_CHECK(m_cursor < m_trace.fields.size() && m_trace.fields[m_cursor].name == name
                    && m_trace.fields[m_cursor].tag == tag && m_trace.fields[m_cursor].components == count,
                "goldenTrace::Recorder: a later sample recorded a field the first sample did not record at this position");
        }
        ++m_cursor;
        m_trace.values.insert(m_trace.values.end(), components, components + count);
    }

    Trace& m_trace;
    std::size_t m_cursor = 0;
    bool m_firstSample = false;
};

inline double floatUlpOf(double magnitude)
{
    const float value = static_cast<float>(std::fabs(magnitude));
    if (!(value < std::numeric_limits<float>::max()))
        return std::numeric_limits<double>::infinity();
    return static_cast<double>(std::nextafter(value, std::numeric_limits<float>::infinity())) - static_cast<double>(value);
}

inline double maxAbsOf(const double* values, uint32_t count)
{
    double largest = 0.0;
    for (uint32_t index = 0; index < count; ++index)
        largest = std::max(largest, std::fabs(values[index]));
    return largest;
}

inline double rotationAngleBetween(const double* first, const double* second)
{
    double dot = 0.0;
    for (uint32_t index = 0; index < 4u; ++index)
        dot += first[index] * second[index];
    const double sign = dot < 0.0 ? -1.0 : 1.0;
    double chordSquared = 0.0;
    for (uint32_t index = 0; index < 4u; ++index)
    {
        const double difference = first[index] - sign * second[index];
        chordSquared += difference * difference;
    }
    return 4.0 * std::asin(std::min(1.0, std::sqrt(chordSquared) * 0.5));
}

inline double fieldMetric(Tag tag, const double* first, const double* second, uint32_t count)
{
    if (tag == Tag::Rotation)
        return rotationAngleBetween(first, second);
    if (tag == Tag::Exact)
    {
        uint32_t differing = 0;
        for (uint32_t index = 0; index < count; ++index)
            differing += first[index] == second[index] ? 0u : 1u;
        return static_cast<double>(differing);
    }
    double largest = 0.0;
    for (uint32_t index = 0; index < count; ++index)
        largest = std::max(largest, std::fabs(first[index] - second[index]));
    return largest;
}

inline double ulpFloorOf(Tag tag, const double* golden, uint32_t count)
{
    const double magnitude = tag == Tag::Rotation ? 1.0 : maxAbsOf(golden, count);
    return double(kUlpFloorFactor) * floatUlpOf(magnitude);
}

enum class BoundClass : uint8_t
{
    Position = 0,
    LinearVelocity = 1,
    Axial = 2,
    Rotation = 3,
    Scalar = 4,
    Planar = 5,
    Discrete = 6,
    Count = 7
};

inline constexpr std::size_t kBoundClassCount = static_cast<std::size_t>(BoundClass::Count);
inline constexpr const char* kBoundClassNames[kBoundClassCount] = { "p.position", "p.velocity", "a", "q", "s", "2", "e" };

inline constexpr double kPositionBoundCm = 1.0;
inline constexpr double kLinearVelocityBoundCmPerS = 33.0;
inline constexpr double kAxialBound = 1e-4;
inline constexpr double kRotationBoundRadians = 1e-5;
inline constexpr double kScalarBound = 1e-5;
inline constexpr double kPlanarBound = 1e-5;
inline constexpr double kDiscreteBound = 0.0;
inline constexpr double kBounds[kBoundClassCount] = { kPositionBoundCm, kLinearVelocityBoundCmPerS, kAxialBound, kRotationBoundRadians,
    kScalarBound, kPlanarBound, kDiscreteBound };

static_assert(kBounds[static_cast<std::size_t>(BoundClass::Position)] == kPositionBoundCm
        && kBounds[static_cast<std::size_t>(BoundClass::LinearVelocity)] == kLinearVelocityBoundCmPerS
        && kBounds[static_cast<std::size_t>(BoundClass::Axial)] == kAxialBound
        && kBounds[static_cast<std::size_t>(BoundClass::Rotation)] == kRotationBoundRadians
        && kBounds[static_cast<std::size_t>(BoundClass::Scalar)] == kScalarBound
        && kBounds[static_cast<std::size_t>(BoundClass::Planar)] == kPlanarBound
        && kBounds[static_cast<std::size_t>(BoundClass::Discrete)] == kDiscreteBound,
    "goldenTrace::kBounds holds one bound per BoundClass, in BoundClass order");

constexpr std::string_view lastNameSegment(std::string_view name)
{
    const std::size_t dot = name.rfind('.');
    return dot == std::string_view::npos ? name : name.substr(dot + 1u);
}

constexpr std::optional<BoundClass> boundClassOf(Tag tag, std::string_view name)
{
    switch (tag)
    {
    case Tag::Position:
    {
        const std::string_view leaf = lastNameSegment(name);
        if (leaf == "position")
            return BoundClass::Position;
        if (leaf == "linearVelocity" || leaf == "velocity")
            return BoundClass::LinearVelocity;
        return std::nullopt;
    }
    case Tag::Direction:
        return std::nullopt;
    case Tag::Axial:
        return BoundClass::Axial;
    case Tag::Rotation:
        return BoundClass::Rotation;
    case Tag::Scalar:
        return BoundClass::Scalar;
    case Tag::Planar:
        return BoundClass::Planar;
    case Tag::Exact:
        return BoundClass::Discrete;
    case Tag::Count:
        break;
    }
    return std::nullopt;
}

static_assert(boundClassOf(Tag::Position, "A.body5.CharacterCapsule.position") == BoundClass::Position
        && boundClassOf(Tag::Position, "A.body5.CharacterCapsule.linearVelocity") == BoundClass::LinearVelocity
        && boundClassOf(Tag::Position, "A.movement.velocity") == BoundClass::LinearVelocity
        && !boundClassOf(Tag::Position, "A.movement.positionOffset").has_value()
        && !boundClassOf(Tag::Position, "A.body0.WeaponAxis.angularVelocity").has_value()
        && !boundClassOf(Tag::Direction, "A.aim.direction").has_value()
        && boundClassOf(Tag::Axial, "A.body0.WeaponAxis.angularVelocity") == BoundClass::Axial
        && boundClassOf(Tag::Exact, "A.machine.currentState") == BoundClass::Discrete,
    "goldenTrace::boundClassOf: a 'p' field is a position or a linear velocity by its last name segment, any other 'p' field and "
    "every 'd' field has no bound, and every other tag is its own class");

template <std::size_t N>
constexpr bool everyFieldHasABound(const FieldRecord (&fields)[N])
{
    for (const FieldRecord& field : fields)
    {
        const std::optional<Tag> tag = tagOfLetter(field.tag);
        if (!tag.has_value() || !boundClassOf(*tag, field.name).has_value())
            return false;
    }
    return true;
}

using EnvelopeRow = std::array<double, kTagCount>;

inline std::vector<EnvelopeRow> envelopeOf(const Trace& base, const std::vector<Trace>& nudged)
{
    std::vector<EnvelopeRow> envelope(base.ticks.size(), EnvelopeRow{});
    for (const Trace& run : nudged)
    {
        OG_CHECK(run.ticks == base.ticks && run.valuesPerSample == base.valuesPerSample && run.fields.size() == base.fields.size(),
            "goldenTrace::envelopeOf: a nudged run must record the same samples and fields as the base run");
        for (std::size_t sample = 0; sample < base.ticks.size(); ++sample)
        {
            for (const FieldSpec& field : base.fields)
            {
                const double metric = fieldMetric(field.tag, base.sample(sample) + field.offset, run.sample(sample) + field.offset,
                    field.components);
                double& slot = envelope[sample][static_cast<std::size_t>(field.tag)];
                slot = std::max(slot, metric);
            }
        }
    }
    return envelope;
}

struct GoldenView
{
    const GoldenHeader* header = nullptr;
    const FieldRecord* fields = nullptr;
    std::size_t fieldCount = 0;
    const uint32_t* ticks = nullptr;
    std::size_t tickCount = 0;
    const double* values = nullptr;
    std::size_t valueCount = 0;
};

struct EnvelopeView
{
    const EnvelopeHeader* header = nullptr;
    const NudgeRecord* nudges = nullptr;
    std::size_t nudgeCount = 0;
    const uint32_t* ticks = nullptr;
    std::size_t tickCount = 0;
    const double* values = nullptr;
    std::size_t valueCount = 0;
};

enum class Mode : uint8_t
{
    Exact,
    Bounds
};

struct ClassMaximum
{
    double metric = 0.0;
    uint32_t tick = 0;
    std::string field;
};

struct CompareResult
{
    bool schemaMatches = false;
    std::string schemaProblem;
    std::size_t comparedSamples = 0;
    std::size_t mismatchingValues = 0;
    std::optional<std::size_t> firstMismatchSample;
    std::string firstMismatch;
    uint32_t ticksPassedBeforeFirstMismatch = 0;
    std::array<double, kTagCount> maxMetric{};
    std::array<double, kTagCount> envelopeRatio{};
    std::array<ClassMaximum, kBoundClassCount> classMaximum{};
    std::array<std::size_t, kTagCount> signedZeroOnly{};
    std::string firstSignedZero;

    bool passed() const { return schemaMatches && mismatchingValues == 0; }
};

inline std::string formatDouble(double value)
{
    char buffer[64];
    std::snprintf(buffer, sizeof(buffer), "%.17g", value);
    return buffer;
}

inline bool sameSchema(const GoldenView& golden, const Trace& live, std::string& problem)
{
    if (golden.fieldCount != live.fields.size())
    {
        problem = "field count " + std::to_string(golden.fieldCount) + " (golden) != " + std::to_string(live.fields.size()) + " (live)";
        return false;
    }
    for (std::size_t index = 0; index < golden.fieldCount; ++index)
    {
        const FieldRecord& expected = golden.fields[index];
        const FieldSpec& actual = live.fields[index];
        if (actual.name != expected.name || tagLetter(actual.tag) != expected.tag || actual.components != expected.components)
        {
            problem = "field " + std::to_string(index) + " is '" + expected.name + "' " + expected.tag + std::to_string(expected.components)
                + " in the golden and '" + actual.name + "' " + tagLetter(actual.tag) + std::to_string(actual.components) + " live";
            return false;
        }
    }
    if (golden.tickCount != live.ticks.size() || !std::equal(live.ticks.begin(), live.ticks.end(), golden.ticks))
    {
        problem = "sample ticks differ (golden " + std::to_string(golden.tickCount) + " samples, live " + std::to_string(live.ticks.size()) + ")";
        return false;
    }
    if (golden.valueCount != live.values.size())
    {
        problem = "value count " + std::to_string(golden.valueCount) + " (golden) != " + std::to_string(live.values.size()) + " (live)";
        return false;
    }
    return true;
}

inline std::string sampleFieldText(const GoldenView& golden, const Trace& live, std::size_t sample, std::size_t field)
{
    return "tick " + std::to_string(live.ticks[sample]) + " field '" + golden.fields[field].name + "' (" + golden.fields[field].tag + ")";
}

inline void noteMismatch(CompareResult& result, const GoldenView& golden, const Trace& live, std::size_t sample, std::size_t field,
    const std::string& detail)
{
    ++result.mismatchingValues;
    if (result.firstMismatchSample.has_value())
        return;
    result.firstMismatchSample = sample;
    result.ticksPassedBeforeFirstMismatch = static_cast<uint32_t>(sample);
    result.firstMismatch = sampleFieldText(golden, live, sample, field) + ": " + detail;
}

inline void noteSignedZero(CompareResult& result, const GoldenView& golden, const Trace& live, std::size_t sample, std::size_t field,
    const std::string& detail)
{
    ++result.signedZeroOnly[static_cast<std::size_t>(live.fields[field].tag)];
    if (result.firstSignedZero.empty())
        result.firstSignedZero = sampleFieldText(golden, live, sample, field) + ": " + detail;
}

constexpr bool sameBitsOrBothZero(double first, double second)
{
    const uint64_t firstBits = std::bit_cast<uint64_t>(first);
    const uint64_t secondBits = std::bit_cast<uint64_t>(second);
    return firstBits == secondBits || ((firstBits | secondBits) << 1u) == 0u;
}

static_assert(sameBitsOrBothZero(0.0, -0.0) && sameBitsOrBothZero(-0.0, 0.0) && sameBitsOrBothZero(-0.0, -0.0)
        && sameBitsOrBothZero(0.25, 0.25) && !sameBitsOrBothZero(0.25, -0.25)
        && !sameBitsOrBothZero(1.0, std::bit_cast<double>(std::bit_cast<uint64_t>(1.0) + 1u))
        && !sameBitsOrBothZero(0.0, double(std::numeric_limits<float>::denorm_min()))
        && !sameBitsOrBothZero(-0.0, -double(std::numeric_limits<float>::denorm_min()))
        && !sameBitsOrBothZero(0.0, std::numeric_limits<double>::denorm_min())
        && sameBitsOrBothZero(std::bit_cast<double>(uint64_t(0x7ff8000000000001u)), std::bit_cast<double>(uint64_t(0x7ff8000000000001u)))
        && !sameBitsOrBothZero(std::bit_cast<double>(uint64_t(0x7ff8000000000001u)), std::bit_cast<double>(uint64_t(0x7ff8000000000002u))),
    "goldenTrace::sameBitsOrBothZero: exact mode equates +0 and -0 and nothing else; a 1-ULP change, a denormal or a NaN payload still differs");

constexpr bool isFiniteBits(double value)
{
    constexpr uint64_t kExponentMask = 0x7ff0000000000000u;
    return (std::bit_cast<uint64_t>(value) & kExponentMask) != kExponentMask;
}

static_assert(!isFiniteBits(std::numeric_limits<double>::infinity()) && !isFiniteBits(-std::numeric_limits<double>::infinity())
        && !isFiniteBits(std::numeric_limits<double>::quiet_NaN()) && !isFiniteBits(std::bit_cast<double>(uint64_t(0xfff8000000000000u)))
        && !isFiniteBits(std::bit_cast<double>(uint64_t(0x7ff0000000000001u)))
        && isFiniteBits(std::numeric_limits<double>::max()) && isFiniteBits(-std::numeric_limits<double>::max())
        && isFiniteBits(0.0) && isFiniteBits(-0.0) && isFiniteBits(std::numeric_limits<double>::denorm_min()),
    "goldenTrace::isFiniteBits: a value is finite unless its exponent field is all ones (+inf, -inf, any NaN); tested on the bits "
    "because /fp:fast may fold std::isfinite, std::isnan and x != x");

inline bool allFiniteBits(const double* values, uint32_t count)
{
    for (uint32_t index = 0; index < count; ++index)
        if (!isFiniteBits(values[index]))
            return false;
    return true;
}

inline bool differsOnlyBySignedZero(const double* first, const double* second, uint32_t count)
{
    for (uint32_t index = 0; index < count; ++index)
        if (!sameBitsOrBothZero(first[index], second[index]))
            return false;
    return true;
}

inline std::string componentsText(const double* values, uint32_t count)
{
    std::string text = "(";
    for (uint32_t index = 0; index < count; ++index)
        text += (index == 0 ? "" : ", ") + formatDouble(values[index]);
    return text + ")";
}

inline CompareResult compareExact(const GoldenView& golden, const Trace& live)
{
    CompareResult result;
    result.schemaMatches = sameSchema(golden, live, result.schemaProblem);
    if (!result.schemaMatches)
        return result;
    for (std::size_t sample = 0; sample < live.ticks.size(); ++sample)
    {
        const double* expected = golden.values + sample * live.valuesPerSample;
        const double* actual = live.sample(sample);
        for (std::size_t field = 0; field < live.fields.size(); ++field)
        {
            const FieldSpec& spec = live.fields[field];
            if (std::memcmp(expected + spec.offset, actual + spec.offset, sizeof(double) * spec.components) != 0)
            {
                const std::string values = "golden " + componentsText(expected + spec.offset, spec.components) + " live "
                    + componentsText(actual + spec.offset, spec.components);
                if (spec.tag != Tag::Exact && differsOnlyBySignedZero(expected + spec.offset, actual + spec.offset, spec.components))
                    noteSignedZero(result, golden, live, sample, field, values);
                else
                    noteMismatch(result, golden, live, sample, field, values + " (bit-for-bit)");
            }
            double& largest = result.maxMetric[static_cast<std::size_t>(spec.tag)];
            largest = std::max(largest, fieldMetric(spec.tag, expected + spec.offset, actual + spec.offset, spec.components));
        }
        ++result.comparedSamples;
    }
    if (!result.firstMismatchSample.has_value())
        result.ticksPassedBeforeFirstMismatch = static_cast<uint32_t>(live.ticks.size());
    return result;
}

inline CompareResult compareBounds(const GoldenView& golden, const EnvelopeView& envelope, const Trace& live)
{
    CompareResult result;
    result.schemaMatches = sameSchema(golden, live, result.schemaProblem);
    if (result.schemaMatches
        && (envelope.tickCount != golden.tickCount || !std::equal(golden.ticks, golden.ticks + golden.tickCount, envelope.ticks)
            || envelope.valueCount != golden.tickCount * kTagCount))
    {
        result.schemaMatches = false;
        result.schemaProblem = "the envelope's samples do not match the golden's";
    }
    for (std::size_t field = 0; result.schemaMatches && field < live.fields.size(); ++field)
    {
        if (!boundClassOf(live.fields[field].tag, live.fields[field].name).has_value())
        {
            result.schemaMatches = false;
            result.schemaProblem = "field '" + live.fields[field].name + "' (" + tagLetter(live.fields[field].tag) + ") has no bound class";
        }
    }
    if (!result.schemaMatches)
        return result;
    for (std::size_t sample = 0; sample < live.ticks.size(); ++sample)
    {
        const double* expected = golden.values + sample * live.valuesPerSample;
        const double* actual = live.sample(sample);
        const double* spread = envelope.values + sample * kTagCount;
        for (std::size_t field = 0; field < live.fields.size(); ++field)
        {
            const FieldSpec& spec = live.fields[field];
            const std::size_t tagIndex = static_cast<std::size_t>(spec.tag);
            const std::size_t classIndex = static_cast<std::size_t>(*boundClassOf(spec.tag, spec.name));
            const bool finite = allFiniteBits(expected + spec.offset, spec.components)
                && allFiniteBits(actual + spec.offset, spec.components);
            const double metric = finite ? fieldMetric(spec.tag, expected + spec.offset, actual + spec.offset, spec.components)
                                         : std::numeric_limits<double>::infinity();
            result.maxMetric[tagIndex] = std::max(result.maxMetric[tagIndex], metric);
            ClassMaximum& largest = result.classMaximum[classIndex];
            if (finite ? metric > largest.metric : isFiniteBits(largest.metric))
                largest = ClassMaximum{ metric, live.ticks[sample], spec.name };
            if (spec.tag != Tag::Exact)
            {
                const double envelopeTolerance = std::max(double(kEnvelopeToleranceFactor) * spread[tagIndex],
                    ulpFloorOf(spec.tag, expected + spec.offset, spec.components));
                result.envelopeRatio[tagIndex] = std::max(result.envelopeRatio[tagIndex], metric / envelopeTolerance);
            }
            const double bound = kBounds[classIndex];
            if (!finite)
            {
                noteMismatch(result, golden, live, sample, field, "golden " + componentsText(expected + spec.offset, spec.components)
                    + " live " + componentsText(actual + spec.offset, spec.components) + " not finite (" + kBoundClassNames[classIndex]
                    + ")");
            }
            else if (!(metric <= bound))
            {
                noteMismatch(result, golden, live, sample, field, "golden " + componentsText(expected + spec.offset, spec.components)
                    + " live " + componentsText(actual + spec.offset, spec.components) + " metric " + formatDouble(metric)
                    + " > bound " + formatDouble(bound) + " (" + kBoundClassNames[classIndex] + "; E(t) "
                    + formatDouble(spread[tagIndex]) + ")");
            }
        }
        ++result.comparedSamples;
    }
    if (!result.firstMismatchSample.has_value())
        result.ticksPassedBeforeFirstMismatch = static_cast<uint32_t>(live.ticks.size());
    return result;
}

inline std::string hexLiteral(double value)
{
    OG_CHECK(isFiniteBits(value), "goldenTrace::hexLiteral: a recorded value must be finite (tested on the bits; was std::isfinite, which /fp:fast may fold)");
    char buffer[64];
    const bool negative = std::signbit(value);
    const std::to_chars_result written = std::to_chars(buffer, buffer + sizeof(buffer), std::fabs(value), std::chars_format::hex);
    OG_CHECK(written.ec == std::errc{}, "goldenTrace::hexLiteral: std::to_chars failed");
    return std::string(negative ? "-0x" : "0x") + std::string(buffer, written.ptr);
}

inline std::string valueLiteral(Tag tag, double value)
{
    if (tag == Tag::Exact && value >= 0.0 && value == std::floor(value) && value < 4294967296.0)
        return std::to_string(static_cast<uint64_t>(value));
    return hexLiteral(value);
}

inline std::string quotedLiteral(const std::string& text)
{
    std::string escaped = "\"";
    for (const char character : text)
    {
        if (character == '"' || character == '\\')
            escaped += '\\';
        escaped += character;
    }
    return escaped + "\"";
}

inline constexpr std::size_t kValuesPerLine = 12u;

inline void appendLine(std::string& out, const std::string& line)
{
    out += line;
    out += "\r\n";
}

inline std::string writeGoldenInl(const Trace& trace, float dtSeconds, uint32_t scenarioTicks, const std::string& sourceCommit,
    const char* generator)
{
    std::string out;
    appendLine(out, "// SPDX-License-Identifier: BUSL-1.1");
    appendLine(out, "// docs/GoldenTraceFormat-rationale.md");
    appendLine(out, "");
    appendLine(out, "#pragma once");
    appendLine(out, "");
    appendLine(out, "namespace goldenTraceData");
    appendLine(out, "{");
    appendLine(out, "");
    appendLine(out, "inline constexpr goldenTrace::GoldenHeader kGoldenHeader{");
    appendLine(out, "    /*formatVersion*/ " + std::to_string(kFormatVersion) + "u,");
    appendLine(out, "    /*axisFrame*/ " + std::to_string(static_cast<uint32_t>(kAxisFrame)) + "u,");
    appendLine(out, "    /*unitsPerMetre*/ " + hexLiteral(double(kUnitsPerMetre)) + "f,");
    appendLine(out, "    /*dtSeconds*/ " + hexLiteral(double(dtSeconds)) + "f,");
    appendLine(out, "    /*scenarioTicks*/ " + std::to_string(scenarioTicks) + "u,");
    appendLine(out, "    /*sourceCommit*/ " + quotedLiteral(sourceCommit) + ",");
    appendLine(out, "    /*generator*/ " + quotedLiteral(generator) + " };");
    appendLine(out, "");
    appendLine(out, "inline constexpr goldenTrace::FieldRecord kGoldenFields[] = {");
    for (const FieldSpec& field : trace.fields)
        appendLine(out, "    { " + quotedLiteral(field.name) + ", '" + tagLetter(field.tag) + "', " + std::to_string(field.components) + "u },");
    appendLine(out, "};");
    appendLine(out, "");
    appendLine(out, "inline constexpr uint32_t kGoldenTicks[] = {");
    for (std::size_t index = 0; index < trace.ticks.size(); index += kValuesPerLine)
    {
        std::string line = "    ";
        for (std::size_t column = index; column < std::min(trace.ticks.size(), index + kValuesPerLine); ++column)
            line += std::to_string(trace.ticks[column]) + "u, ";
        line.pop_back();
        appendLine(out, line);
    }
    appendLine(out, "};");
    appendLine(out, "");
    appendLine(out, "inline constexpr double kGoldenValues[] = {");
    for (std::size_t sample = 0; sample < trace.ticks.size(); ++sample)
    {
        const double* values = trace.sample(sample);
        std::string line = "    ";
        std::size_t onLine = 0;
        for (const FieldSpec& field : trace.fields)
        {
            for (uint32_t component = 0; component < field.components; ++component)
            {
                line += valueLiteral(field.tag, values[field.offset + component]) + ", ";
                if (++onLine == kValuesPerLine)
                {
                    line.pop_back();
                    appendLine(out, line);
                    line = "    ";
                    onLine = 0;
                }
            }
        }
        if (onLine != 0)
        {
            line.pop_back();
            appendLine(out, line);
        }
    }
    appendLine(out, "};");
    appendLine(out, "");
    appendLine(out, "} // namespace goldenTraceData");
    return out;
}

inline std::string writeEnvelopeInl(const std::vector<uint32_t>& ticks, const std::vector<EnvelopeRow>& envelope,
    const std::vector<Nudge>& nudges, const std::string& sourceCommit)
{
    std::string out;
    appendLine(out, "// SPDX-License-Identifier: BUSL-1.1");
    appendLine(out, "// docs/GoldenTraceFormat-rationale.md");
    appendLine(out, "");
    appendLine(out, "#pragma once");
    appendLine(out, "");
    appendLine(out, "namespace goldenTraceEnvelopeData");
    appendLine(out, "{");
    appendLine(out, "");
    appendLine(out, "inline constexpr goldenTrace::EnvelopeHeader kEnvelopeHeader{");
    appendLine(out, "    /*formatVersion*/ " + std::to_string(kFormatVersion) + "u,");
    appendLine(out, "    /*runs*/ " + std::to_string(nudges.size()) + "u,");
    appendLine(out, "    /*axisFrame*/ " + std::to_string(static_cast<uint32_t>(kAxisFrame)) + "u,");
    appendLine(out, "    /*unitsPerMetre*/ " + hexLiteral(double(kUnitsPerMetre)) + "f,");
    appendLine(out, "    /*sourceCommit*/ " + quotedLiteral(sourceCommit) + " };");
    appendLine(out, "");
    appendLine(out, "inline constexpr goldenTrace::NudgeRecord kEnvelopeNudges[] = {");
    for (const Nudge& nudge : nudges)
        appendLine(out, "    { " + std::to_string(nudge.characterId) + "u, " + std::to_string(nudge.component) + "u, "
            + std::to_string(nudge.direction) + " },");
    appendLine(out, "};");
    appendLine(out, "");
    appendLine(out, "inline constexpr uint32_t kEnvelopeTicks[] = {");
    for (std::size_t index = 0; index < ticks.size(); index += kValuesPerLine)
    {
        std::string line = "    ";
        for (std::size_t column = index; column < std::min(ticks.size(), index + kValuesPerLine); ++column)
            line += std::to_string(ticks[column]) + "u, ";
        line.pop_back();
        appendLine(out, line);
    }
    appendLine(out, "};");
    appendLine(out, "");
    std::string tagOrder;
    for (std::size_t tag = 0; tag < kTagCount; ++tag)
        tagOrder += kTagLetters[tag];
    appendLine(out, "inline constexpr char kEnvelopeTagOrder[] = " + quotedLiteral(tagOrder) + ";");
    appendLine(out, "");
    appendLine(out, "inline constexpr double kEnvelopeValues[] = {");
    for (const EnvelopeRow& row : envelope)
    {
        std::string line = "    ";
        for (std::size_t tag = 0; tag < kTagCount; ++tag)
            line += hexLiteral(row[tag]) + ", ";
        line.pop_back();
        appendLine(out, line);
    }
    appendLine(out, "};");
    appendLine(out, "");
    appendLine(out, "} // namespace goldenTraceEnvelopeData");
    return out;
}

} // namespace goldenTrace
