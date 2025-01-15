# ros2_streamlit_openai
A minimal integration of streamlit web chat interface with OpenAI API wrapped in ROS2 nodes, for natural language interface and publication of queries and completions on ROS2 topics. Tested with ROS2 Jazzy on Ubuntu 24.04. Uses sockets for interprocess communication with Streamlit UI.

Set the OpenAI key in your local environments

```
export OPENAI_API_KEY=sk-123abc
```

Clone the repository and ```colcon build``` from the top of the directory.

Customize the Streamlit UI and extend to your heart's content.
