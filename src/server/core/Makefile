server: ClientConnection.cc ClientConnection.h ClientHandler.h \
		CloudFilesystemView.h CloudFilesystemView.cc FilesystemClientHandler.h \
		FilesystemClientHandler.cc JobClientHandler.cc JobClientHandler.h \
		ServerListener.cc ServerListener.h SpookyHash.h SpookyHash.cc json.hpp \
		server.cc
	g++ -lpthread -pthread -fpermissive -std=c++11 -Wall ClientConnection.cc CloudFilesystemView.cc\
		FilesystemClientHandler.cc JobClientHandler.cc ServerListener.cc \
		SpookyHash.cc server.cc -o server

initialize: initialize.cc SpookyHash.cc SpookyHash.h CloudFilesystemView.cc \
		CloudFilesystemView.h
	g++ --std=c++11 -Wall initialize.cc SpookyHash.cc -o initialize
