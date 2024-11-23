from rclpy.node import Node

class Tool:
    def call(self, node: Node, values: dict):
        raise NotImplementedError

    @staticmethod
    def discover(node: Node, node_name: str, node_namespace: str, rules: list[str] = None):
        raise NotImplementedError
    
    @staticmethod
    def _match_rules(rules: list[str], name: str) -> bool:
        """
        Checks if a name matches the passed rules.
        Rules are general include rules but they can be inverted by prefixing them with a '!'.

        Rules can contain wildcards pre-/suffixes (*) to match multiple names.
        

        Args:
            rules: A list of rules to match against.
            name: The name to match against.
        """
        exclude_rules = [rule[1:] for rule in rules if rule.startswith('!')]
        include_rules = [rule for rule in rules if not rule.startswith('!')]

        if len(include_rules) > 0:
            if not any([Tool._match_rule(rule, name) for rule in include_rules]):
                return False
            
        if len(exclude_rules) > 0:
            if any([Tool._match_rule(rule, name) for rule in exclude_rules]):
                return False
            
        return True
    
    @staticmethod
    def _match_rule(rule: str, name: str) -> bool:
        """
        Checks if a name matches the passed rule.
        Rules are general include rules but they can be inverted by prefixing them with a '!'.

        Rules can contain wildcards pre-/suffixes (*) to match multiple names.
        

        Args:
            rule: A rule to match against.
            name: The name to match against.
        """
        if rule.endswith('*'):
            return name.startswith(rule[:-1])
        if rule.startswith('*'):
            return name.endswith(rule[1:])
        return rule == name