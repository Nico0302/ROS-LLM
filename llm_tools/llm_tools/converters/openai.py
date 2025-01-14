from typing import Optional
from rosidl_adapter import parser
from llm_tools.converters.converter import Converter
from llm_tools.tool import Tool
import copy
import json
                                                                            
class OpenAIConverter(Converter):
    """
    Converts a ROS service specification into a openai function definition.
    """

    def convert_tools(self, tools: list[Tool]) -> str:
        tool_definitions = []
        for tool in tools:
            tool_definitions.append(tool.accept(self))
        return json.dumps(tool_definitions)

    def visit_service(self, service):
        """
        Accepts a service and converts it into a openai function definition.
        """
        return self.convert_service(service.type, service.get_names()[0])
    
    def visit_topic(self, topic):
        """
        Accepts a topic and converts it into a openai message definition.
        """
        function = { 
            "name": topic.get_names()[0],
            "parameters": {
                "type": "object",
                "properties": {
                    "values": self.convert_message(topic.type),
                    "times": { "type": "integer", "minimum": 1, "default": 1, "description": "The number of times to publish the message with 1 Hz (number of steps)." }
                },
                "required": ["values"]
            },
            "description": "Publishes a message to the topic " + topic.name
        }
    
        return {
            'type': 'function',
            'function': function
        }
    
    def convert_type(self, type: parser.Type | parser.MessageSpecification):
        """
        Converts a ROS type into a openai type.
        """
        if isinstance(type, parser.MessageSpecification):
            return self.convert_message(type)
        elif type.is_array:
            return self.convert_array_type(type)
        elif type.is_primitive_type():
            return self.convert_primitive_type(type)
        else:
            # load the external message specification
            return self.convert_message((type.pkg_name or "") + "/msg/" + type.type)
        
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
        return [
            field.name, 
            self.convert_type(field.type) | 
            self._get_description(field) | 
            ({ "default": field.default_value } if field.default_value is not None else {})
        ] 
    
    def convert_message(self, spec: parser.MessageSpecification | str, parseDescription: bool = True):
        """
        Converts a ROS message specification into a openai message definition.
        """
        if isinstance(spec, str):
            spec = parser.parse_message_file(*self._get_interface_path(spec))
        properties = {}
        required = []
        for field in spec.fields:
            [name, definiton] = self.convert_field(field)
            properties[name] = definiton
            if definiton.get("default") is None and "optional" not in definiton.get("description", ""):
                required.append(name)
        return { "type": "object", "properties": properties } | \
            (self._get_description(spec) if parseDescription else {}) | \
            ({ "required": required } if len(required) > 0 else {})
    
    def convert_service(self, spec: parser.ServiceSpecification | str, name: Optional[str] = None):
        """
        Converts a ROS service specification into a openai function definition.
        """
        if isinstance(spec, str):
            spec = parser.parse_service_file(*self._get_interface_path(spec))
        request = self.convert_message(spec.request, False)
        
        function = { 
            "name": spec.srv_name if name is None else name,
            "parameters": request,
        } | self._get_description(spec.request)
    
        return {
            'type': 'function',
            'function': function
        }
    
    def _get_description(self, spec):
        """
        Get the description from the annotations.
        """
        if "comment" in spec.annotations and len(spec.annotations["comment"]) > 0:
            return { "description": "\n".join(spec.annotations["comment"]) }
        return {}
    

def main():
    converter = OpenAIConverter()
    interface_name = "geometry_msgs/msg/Twist"
    print(converter.convert_message(interface_name))

if __name__ == '__main__':
    main()