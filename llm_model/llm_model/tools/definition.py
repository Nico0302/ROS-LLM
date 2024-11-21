from rosidl_adapter import parser
from rosidl_runtime_py import get_interface_path
import copy

"""
Converts a ROS service specification into a openai function definition.
"""
class DefinitionGenerator:
    """
    Generates a JSON Schema function definition from a ROS srv interface.
    """
    
    def convert_type(self, type: parser.Type | parser.MessageSpecification):
        """
        Converts a ROS type into a openai type.
        """
        if isinstance(type, parser.ServiceSpecification):
            return self.convert_service(type)
        elif isinstance(type, parser.MessageSpecification):
            return self.convert_message(type)
        elif type.is_array:
            return self.convert_array_type(type)
        elif type.is_primitive_type():
            return self.convert_primitive_type(type)
        else:
            # load the external message specification
            return self.convert_message(type.pkg_name + "/msg/" + type.type)
        
    def convert_primitive_type(self, type: parser.Type):
        """
        Converts a ROS primitive type into a openai type.
        """
        match type.type:
            case "bool":
                return { "type": "boolean" }
            case "int8" | "int16" | "int32" | "int64":
                return { "type": "integer" }
            case "uint8" | "uint16" | "uint32" | "uint64":
                return { "type": "integer", "minimum": 0 }
            case "float32" | "float64":
                return { "type": "number" }
            case "string" | "wstring":
                return { "type": "string" }
            case "char":
                return { "type": "string", "minLength": 1, "maxLength": 1 }
            case _:
                return { "type": "null" }
            
    def convert_array_type(self, type: parser.Type):
        """
        Converts a ROS array type into a openai type.
        """
        item_type = copy.deepcopy(type)
        item_type.is_array = False
        definiton = { "type": "array", "items": self.convert_type(item_type) }
        if type.is_fixed_size_array(): 
            definiton["minItems"] = type.array_size
            definiton["maxItems"] = type.array_size
        if type.is_upper_bound:
            definiton["maxItems"] = type.array_size
        return definiton
    
    def convert_field(self, field: parser.Field):
        """
        Converts a ROS field into a openai field.
        """
        return [field.name, self.convert_type(field.type) | self._get_description(field)] 
    
    def convert_message(self, spec: parser.MessageSpecification | str):
        """
        Converts a ROS message specification into a openai message definition.
        """
        if isinstance(spec, str):
            spec = parser.parse_message_file(*self._get_interface_path(spec))
        properties = {}
        for field in spec.fields:
            [name, definiton] = self.convert_field(field)
            properties[name] = definiton
        return { "type": "object", "properties": properties } | self._get_description(spec)
    
    def convert_service(self, spec: parser.ServiceSpecification | str):
        """
        Converts a ROS service specification into a openai function definition.
        """
        if isinstance(spec, str):
            spec = parser.parse_service_file(*self._get_interface_path(spec))
        request = self.convert_message(spec.request)
        function = { 
            "name": spec.srv_name,
            "parameters": request,
        } | self._get_description(spec.request),
        return {
            "type": "function",
            "function": function
        }
    
    def _get_description(self, spec):
        """
        Get the description from the annotations.
        """
        if "comment" in spec.annotations and len(spec.annotations["comment"]) > 0:
            return { "description": "\n".join(spec.annotations["comment"]) }
        return {}

    def _get_interface_path(self, interface_name: str):
        [package, _, _] = interface_name.rsplit("/")
        return [package, get_interface_path(interface_name)]
    

def main():
    converter = DefinitionGenerator()
    interface_name = "geometry_msgs/msg/Twist"
    print(converter.convert_message(interface_name))

if __name__ == '__main__':
    main()