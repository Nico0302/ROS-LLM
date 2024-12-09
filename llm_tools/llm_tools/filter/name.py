from llm_tools.topic import Topic
from llm_tools.filter.filter import Filter
from llm_tools.tool import Tool
from llm_tools.service import Service
from dataclasses import dataclass
import fnmatch

@dataclass
class NameFilter(Filter):

    include_services: list[str]
    exclude_services: list[str]

    include_topics: list[str]
    exclude_topics: list[str]

    def visit_service(self, service: Service):
        return self._match_one(service.name, self.include_services) and \
        not self._match_one(service.name, self.exclude_services)
    
    def visit_topic(self, topic: Topic):
        return self._match_one(topic.name, self.include_topics) and \
        not self._match_one(topic.name, self.exclude_topics)
    
    def _match_one(self, name, list):
        for pat in list:
            if fnmatch.fnmatch(name, pat):
                return True 
        return False