import enum
import struct
import typing as T
from dataclasses import dataclass
from collections import defaultdict

MessageSchema = T.Union['StructSchema', 'PrimitiveSchema']

@dataclass(frozen=True)
class StructSchema:
    fields: T.Dict[str, MessageSchema]

class PrimitiveSchema(enum.Enum):
    INT = 'int'
    LONG = 'long'
    DOUBLE = 'double'
    STRING = 'string'
    BOOLEAN = 'boolean'

@dataclass(frozen=True)
class EnumSchema:
    constants: T.List[str]


def read_file(f):
    def read(n):
        assert n > 0
        # assume this reads exactly n bytes or we reach EOF
        buf = f.read(n)
        if len(buf) == 0:
            raise EOFError
        if len(buf) < n:
            raise IOError('Short read')
        return buf

    def read_string():
        nbytes, = struct.unpack_from('!i', read(4))
        name, = struct.unpack_from(f'!{nbytes}s', read(nbytes))
        return name.decode('utf-8')

    def read_schema():
        schema_type, = struct.unpack_from('!i', read(4))
        # struct schema
        if schema_type == 0:
            nfields, = struct.unpack_from('!i', read(4))
            fields = {}
            for _ in range(nfields):
                name = read_string()
                fields[name] = read_schema()
            return StructSchema(fields)
        # primitive schema
        elif schema_type == 1:
            return PrimitiveSchema.INT
        elif schema_type == 2:
            return PrimitiveSchema.LONG
        elif schema_type == 3:
            return PrimitiveSchema.DOUBLE
        elif schema_type == 4:
            return PrimitiveSchema.STRING
        elif schema_type == 5:
            return PrimitiveSchema.BOOLEAN
        # enum schema
        elif schema_type == 6:
            nconstants, = struct.unpack_from('!i', read(4))
            constants = []
            for _ in range(nconstants):
                constants.append(read_string())
            return EnumSchema(constants)
        else:
            raise ValueError(f'Unknown schema type: {schema_type}')

    def read_msg(schema):
        if isinstance(schema, StructSchema):
            msg = {}
            for name, field_schema in schema.fields.items():
                msg[name] = read_msg(field_schema)
            return msg
        elif isinstance(schema, PrimitiveSchema):
            if schema == PrimitiveSchema.INT:
                return struct.unpack_from('!i', read(4))[0]
            elif schema == PrimitiveSchema.LONG:
                return struct.unpack_from('!q', read(8))[0]
            elif schema == PrimitiveSchema.DOUBLE:
                return struct.unpack_from('!d', read(8))[0]
            elif schema == PrimitiveSchema.STRING:
                return read_string()
            elif schema == PrimitiveSchema.BOOLEAN:
                return struct.unpack_from('!?', read(1))[0]
            else:
                raise ValueError(f'Unknown primitive schema: {schema}')
        elif isinstance(schema, EnumSchema):
            ordinal, = struct.unpack_from('!i', read(4))
            return schema.constants[ordinal]
        else:
            raise ValueError(f'Unknown schema: {schema}')


    magic, version = struct.unpack_from('!2sh', read(4))
    assert magic == b'RR'
    assert version == 0

    channels = []
    schemas = {}
    messages = defaultdict(list)

    while True:
        try:
            entry_type, = struct.unpack_from('!i', read(4))
            if entry_type == 0:
                # channel definition
                ch = read_string()
                schemas[ch] = read_schema()
                channels.append(ch)
            elif entry_type == 1:
                # message
                ch_index, = struct.unpack_from('!i', read(4))
                ch = channels[ch_index]
                messages[ch].append(read_msg(schemas[ch]))
            else:
                raise ValueError(f"Unknown entry type: {entry_type}")

        except EOFError:
            break

    return schemas, dict(messages)


if __name__ == '__main__':
    import sys
    filepath = sys.argv[1]
    with open(filepath, 'rb') as f:
        schemas, messages = read_file(f)
        for ch, schema in schemas.items():
            print(f'Channel: {ch}  ({len(messages[ch])} messages)\n  {schema}')