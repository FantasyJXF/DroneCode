/*
 * UAVCAN data structure definition for libuavcan.
 *
 * Autogenerated, do not edit.
 *
 * Source file: /home/fantasy/Documents/DroneCode/src/modules/uavcan/libuavcan/dsdl/uavcan/equipment/indication/SingleLightCommand.uavcan
 */

#ifndef UAVCAN_EQUIPMENT_INDICATION_SINGLELIGHTCOMMAND_HPP_INCLUDED
#define UAVCAN_EQUIPMENT_INDICATION_SINGLELIGHTCOMMAND_HPP_INCLUDED

#include <uavcan/build_config.hpp>
#include <uavcan/node/global_data_type_registry.hpp>
#include <uavcan/marshal/types.hpp>

#include <uavcan/equipment/indication/RGB565.hpp>

/******************************* Source text **********************************
#
# Nested type.
# Controls single light source, color or monochrome.
#

uint8 light_id

RGB565 color      # Monocolor lights should interpret this as brightness
******************************************************************************/

/********************* DSDL signature source definition ***********************
uavcan.equipment.indication.SingleLightCommand
saturated uint8 light_id
uavcan.equipment.indication.RGB565 color
******************************************************************************/

#undef light_id
#undef color

namespace uavcan
{
namespace equipment
{
namespace indication
{

template <int _tmpl>
struct UAVCAN_EXPORT SingleLightCommand_
{
    typedef const SingleLightCommand_<_tmpl>& ParameterType;
    typedef SingleLightCommand_<_tmpl>& ReferenceType;

    struct ConstantTypes
    {
    };

    struct FieldTypes
    {
        typedef ::uavcan::IntegerSpec< 8, ::uavcan::SignednessUnsigned, ::uavcan::CastModeSaturate > light_id;
        typedef ::uavcan::equipment::indication::RGB565 color;
    };

    enum
    {
        MinBitLen
            = FieldTypes::light_id::MinBitLen
            + FieldTypes::color::MinBitLen
    };

    enum
    {
        MaxBitLen
            = FieldTypes::light_id::MaxBitLen
            + FieldTypes::color::MaxBitLen
    };

    // Constants

    // Fields
    typename ::uavcan::StorageType< typename FieldTypes::light_id >::Type light_id;
    typename ::uavcan::StorageType< typename FieldTypes::color >::Type color;

    SingleLightCommand_()
        : light_id()
        , color()
    {
        ::uavcan::StaticAssert<_tmpl == 0>::check();  // Usage check

#if UAVCAN_DEBUG
        /*
         * Cross-checking MaxBitLen provided by the DSDL compiler.
         * This check shall never be performed in user code because MaxBitLen value
         * actually depends on the nested types, thus it is not invariant.
         */
        ::uavcan::StaticAssert<24 == MaxBitLen>::check();
#endif
    }

    bool operator==(ParameterType rhs) const;
    bool operator!=(ParameterType rhs) const { return !operator==(rhs); }

    /**
     * This comparison is based on @ref uavcan::areClose(), which ensures proper comparison of
     * floating point fields at any depth.
     */
    bool isClose(ParameterType rhs) const;

    static int encode(ParameterType self, ::uavcan::ScalarCodec& codec,
                      ::uavcan::TailArrayOptimizationMode tao_mode = ::uavcan::TailArrayOptEnabled);

    static int decode(ReferenceType self, ::uavcan::ScalarCodec& codec,
                      ::uavcan::TailArrayOptimizationMode tao_mode = ::uavcan::TailArrayOptEnabled);

    /*
     * Static type info
     */
    enum { DataTypeKind = ::uavcan::DataTypeKindMessage };
    // This type has no default data type ID

    static const char* getDataTypeFullName()
    {
        return "uavcan.equipment.indication.SingleLightCommand";
    }

    static void extendDataTypeSignature(::uavcan::DataTypeSignature& signature)
    {
        signature.extend(getDataTypeSignature());
    }

    static ::uavcan::DataTypeSignature getDataTypeSignature();

};

/*
 * Out of line struct method definitions
 */

template <int _tmpl>
bool SingleLightCommand_<_tmpl>::operator==(ParameterType rhs) const
{
    return
        light_id == rhs.light_id &&
        color == rhs.color;
}

template <int _tmpl>
bool SingleLightCommand_<_tmpl>::isClose(ParameterType rhs) const
{
    return
        ::uavcan::areClose(light_id, rhs.light_id) &&
        ::uavcan::areClose(color, rhs.color);
}

template <int _tmpl>
int SingleLightCommand_<_tmpl>::encode(ParameterType self, ::uavcan::ScalarCodec& codec,
    ::uavcan::TailArrayOptimizationMode tao_mode)
{
    (void)self;
    (void)codec;
    (void)tao_mode;
    int res = 1;
    res = FieldTypes::light_id::encode(self.light_id, codec,  ::uavcan::TailArrayOptDisabled);
    if (res <= 0)
    {
        return res;
    }
    res = FieldTypes::color::encode(self.color, codec,  tao_mode);
    return res;
}

template <int _tmpl>
int SingleLightCommand_<_tmpl>::decode(ReferenceType self, ::uavcan::ScalarCodec& codec,
    ::uavcan::TailArrayOptimizationMode tao_mode)
{
    (void)self;
    (void)codec;
    (void)tao_mode;
    int res = 1;
    res = FieldTypes::light_id::decode(self.light_id, codec,  ::uavcan::TailArrayOptDisabled);
    if (res <= 0)
    {
        return res;
    }
    res = FieldTypes::color::decode(self.color, codec,  tao_mode);
    return res;
}

/*
 * Out of line type method definitions
 */
template <int _tmpl>
::uavcan::DataTypeSignature SingleLightCommand_<_tmpl>::getDataTypeSignature()
{
    ::uavcan::DataTypeSignature signature(0x945D0D4A16EE764EULL);

    FieldTypes::light_id::extendDataTypeSignature(signature);
    FieldTypes::color::extendDataTypeSignature(signature);

    return signature;
}

/*
 * Out of line constant definitions
 */

/*
 * Final typedef
 */
typedef SingleLightCommand_<0> SingleLightCommand;

// No default registration

} // Namespace indication
} // Namespace equipment
} // Namespace uavcan

/*
 * YAML streamer specialization
 */
namespace uavcan
{

template <>
class UAVCAN_EXPORT YamlStreamer< ::uavcan::equipment::indication::SingleLightCommand >
{
public:
    template <typename Stream>
    static void stream(Stream& s, ::uavcan::equipment::indication::SingleLightCommand::ParameterType obj, const int level);
};

template <typename Stream>
void YamlStreamer< ::uavcan::equipment::indication::SingleLightCommand >::stream(Stream& s, ::uavcan::equipment::indication::SingleLightCommand::ParameterType obj, const int level)
{
    (void)s;
    (void)obj;
    (void)level;
    if (level > 0)
    {
        s << '\n';
        for (int pos = 0; pos < level; pos++)
        {
            s << "  ";
        }
    }
    s << "light_id: ";
    YamlStreamer< ::uavcan::equipment::indication::SingleLightCommand::FieldTypes::light_id >::stream(s, obj.light_id, level + 1);
    s << '\n';
    for (int pos = 0; pos < level; pos++)
    {
        s << "  ";
    }
    s << "color: ";
    YamlStreamer< ::uavcan::equipment::indication::SingleLightCommand::FieldTypes::color >::stream(s, obj.color, level + 1);
}

}

namespace uavcan
{
namespace equipment
{
namespace indication
{

template <typename Stream>
inline Stream& operator<<(Stream& s, ::uavcan::equipment::indication::SingleLightCommand::ParameterType obj)
{
    ::uavcan::YamlStreamer< ::uavcan::equipment::indication::SingleLightCommand >::stream(s, obj, 0);
    return s;
}

} // Namespace indication
} // Namespace equipment
} // Namespace uavcan

#endif // UAVCAN_EQUIPMENT_INDICATION_SINGLELIGHTCOMMAND_HPP_INCLUDED