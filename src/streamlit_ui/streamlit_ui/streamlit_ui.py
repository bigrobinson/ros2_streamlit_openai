import os
import socket
import threading
import streamlit as st
from streamlit.runtime.scriptrunner import add_script_run_ctx

@st.cache_resource
def create_server(socket_path):
    # remove the socket file if it already exists
    try:
        os.unlink(socket_path)
    except OSError:
        if os.path.exists(socket_path):
            raise
    # Create the Unix socket server
    server = socket.socket(socket.AF_UNIX, socket.SOCK_STREAM)
    # Bind the socket to the path
    server.bind(socket_path)
    # Listen for incoming connections
    server.listen(2)

    return server


@st.cache_resource
def connect_clients(_server):
    index=1
    # Initialize dict for pub and sub client connections.
    connections = {'publisher':None, 'subscriber':None}

    for _ in range(2):
        # Create streamlit notification
        placeholder = st.empty()
        with placeholder.container():
            st.write(f'Server is listening for connection {index}...')
        # accept connections
        connection, _ = _server.accept()
        # receive identifier message
        try:
            identifier = connection.recv(1024).decode()
            print(f"Connected to {identifier}")
        except socket.error as e:
            print(f"Socket error: {e}")
            return
        except UnicodeDecodeError as e:
            print(f"Decode error: {e}")
            return

        # Clear streamlit notification
        placeholder.empty()
        connections[identifier] = connection
        index += 1

    return connections


@st.cache_data
def get_avatars():
    avatars = {'user': None, 'assistant': None}
    avatars['user'] = os.path.join(os.path.expanduser('~'),'ros2_ws/src/streamlit_ui/resource/loki.jpg')
    avatars['assistant'] = os.path.join(os.path.expanduser('~'),'ros2_ws/src/streamlit_ui/resource/robopirate.jpg')
    return avatars

def handle_publisher(publisher_connection):
    # React to user input
    if prompt := st.chat_input('enter a string'):
        # Display user message in chat message container
        st.chat_message('user', avatar=get_avatars()['user']).markdown(prompt)
        # Add user message to chat history
        st.session_state.messages.append({'role': 'user', 'content': prompt})
        # Primitive callback for the button
        publisher_connection.sendall(prompt.encode())


def handle_subscriber(subscriber_connection):
    # React to chat completion
    if response := subscriber_connection.recv(1024).decode():
        # Display assistant message
        st.chat_message('assistant', avatar=get_avatars()['assistant']).markdown(response)
        # Add assistant message to the chat history
        st.session_state.messages.append({'role':'assistant', 'content': response})



def main():

    # Set up the main layout
    st.title('One day I am going to be a real robot.')

    # Initialize chat history
    if 'messages' not in st.session_state:
        st.session_state.messages = []
        st.session_state.messages.append({'role': 'assistant', 'content': 'Hello, human.'})

    # Display chat messages from history on rerun
    for message in st.session_state.messages:
        with st.chat_message(message['role'], avatar=get_avatars()[message['role']]):
            st.markdown(message['content'])

    # Create server socket and connection to clients.
    socket_path = '/tmp/st_socket.socket'
    server = create_server(socket_path)
    connections = connect_clients(server)

    # Start threads for publisher and subscriber clients
    publisher_thread  = threading.Thread(target=handle_publisher, args=(connections['publisher'],))
    add_script_run_ctx(publisher_thread)
    publisher_thread.start()

    subscriber_thread = threading.Thread(target=handle_subscriber, args=(connections['subscriber'],))
    add_script_run_ctx(subscriber_thread)
    subscriber_thread.start()

    # Join threads
    publisher_thread.join()
    subscriber_thread.join()



if __name__ == "__main__":
    main()
