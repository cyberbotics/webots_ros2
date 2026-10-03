import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import openai
import gradio as gr
import argparse

parser = argparse.ArgumentParser()
parser.add_argument("--prompt", type=str, default="prompts/mavic_basic.txt")
args = parser.parse_args()

class LLMIntegrationNode(Node):
    def __init__(self):
        super().__init__('llm_integration_node')

        # Publisher untuk mengirim respons LLM ke parser
        self.command_publisher = self.create_publisher(String, '/llm_to_drone_command', 10)

        # Subscriber untuk menerima perintah dari pengguna
        self.user_command_subscription = self.create_subscription(
            String,
            '/user_command',
            self.process_user_command,
            10
        )

        # Set API key OpenAI
        openai.api_key = "API_Key"  

    def process_user_command(self, msg):
        user_query = msg.data
        self.get_logger().info(f"Received user command: {user_query}")

        # Kirim perintah ke LLM dan dapatkan respons
        self.llm_response = self.query_llm(user_query)
        if self.llm_response:
            self.get_logger().info(f"LLM Response: {self.llm_response}")

            # Publikasikan respons ke topic /llm_to_drone_command
            self.command_publisher.publish(String(data=self.llm_response))
        else:
            self.get_logger().error("Failed to get a response from LLM.")
    
        
    
    def query_llm(self, user_query):
        try:
            response = openai.ChatCompletion.create(
                model="gpt-4o",
                messages=[
                    {
                        "role": "system",
                        "content": prompt,
                    },
                    {"role": "user", "content": user_query},
                ]
            )
            return response['choices'][0]['message']['content']
        except Exception as e:
            self.get_logger().error(f"Error querying LLM: {e}")
            return None

    def send_prompt(self, prompt):
        # Publikasikan prompt ke topik /user_command
        
        msg = String()
        msg.data = prompt
        self.user_command_subscription.callback(msg)
        self.get_logger().info(f"Sent Prompt: {prompt}")
    
    def process_prompt(self, prompt: str):
        # Fungsi untuk mengolah input prompt (misalnya memanggil send_prompt)
        self.send_prompt(prompt)
        return self.llm_response
    
with open(args.prompt, "r") as f:
    prompt = f.read()   

def main(args=None):
    rclpy.init(args=args)
    node = LLMIntegrationNode()
    # Fungsi untuk menerima input dari Gradio dan memprosesnya
    def gradio_interface(prompt):
        response=node.process_prompt(prompt)
        return response

    # Membuat antarmuka Gradio
    interface = gr.Interface(
        fn=gradio_interface,  # Fungsi yang akan dipanggil
        inputs="text",        # Input berupa teks
        outputs="text",       # Output berupa teks
    )

    # Menjalankan UI Gradio di thread terpisah
    interface.launch(share=True)
    
    try:
        # node.send_prompt(prompt)
        # prompt = "Create waypoints in a good circular shape with the center at x:10, y:10, z:5, a radius of 5, altitude 10, customize the number of waypoints with the shape."
        
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

