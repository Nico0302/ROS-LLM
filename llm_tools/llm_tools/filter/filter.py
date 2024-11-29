from llm_tools.tool import Tool, ToolVisitor

class Filter(ToolVisitor):

    def filter_tools(self, tools: list[Tool]) -> list[Tool]:
        filtered_tools = []
        for tool in tools:
            if tool.accept(self):
                filtered_tools.append(tool)
        return filtered_tools