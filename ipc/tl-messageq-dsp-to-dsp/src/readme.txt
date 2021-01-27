#
#  ======== readme.txt ========
#

hello - Send one-way messages from writer to reader

Overview
=========================================================================
This is the "Hello World" example for IPC. It is a very simple example
to help you get started. It is a two processor example. You can build
it for any two processors on your device, but only for two at a time.

It uses the reader/writer design pattern. One processor will be the reader
and the other will be the writer. The reader creates a message queue and
waits on it for messages. The writer opens the reader's message queue
and sends messages to it. The writer allocates the message from a shared
message pool and the reader returns the message to the pool. Thus, messages
are sent in only one direction, but they are recycled back to the pool.
