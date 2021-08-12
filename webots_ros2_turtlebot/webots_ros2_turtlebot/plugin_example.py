class PluginExample:
    def init(self, webots_node, properties):
        print('init()')
        print(properties)
        webots_node.test()

    def step(self):
        print('step()')
