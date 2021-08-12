class PluginExample:
    def init(self, webots_node):
        print('init()')
        webots_node.test()

    def step(self):
        print('step()')
