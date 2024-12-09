from llm_tools.tool import Tool, ToolVisitor

class Filter(ToolVisitor):

    def filter_tools(self, tools: list[Tool]) -> list[Tool]:
        filtered_tools = []
        for tool in tools:
            if tool.accept(self):
                # append only if the tool is not already in the list
                if not any(t == tool for t in filtered_tools):
                    filtered_tools.append(tool)
        return filtered_tools