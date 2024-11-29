from llm_tools.filter.filter import Filter
from llm_tools.tool import Tool
from llm_tools.service import Service
import fnmatch

class NameFilter(Filter):
    def __init__(self, include_services: list[str] = [], exclude_services: list[str] = []):
        self.include_services = include_services
        self.exclude_services = exclude_services

    def visit_service(self, service: Service):
        return self._match_one(service.name, self.include_services) and \
        not self._match_one(service.name, self.exclude_services)
    
    def _match_one(self, name, list):
        for pat in list:
            if fnmatch.fnmatch(name, pat):
                return True 
        return False