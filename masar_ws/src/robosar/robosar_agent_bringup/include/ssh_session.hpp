// Created by Indraneel on 25/01/21

#ifndef SSH_SESSION_HPP
#define SSH_SESSION_HPP

#include <iostream>
#include <string>
#include <libssh/libsshpp.hpp>

class SSHSession
{

public:
    SSHSession(const std::string ip_address)
    {
        session.setOption(SSH_OPTIONS_HOST, &ip_address[0]);
    }

    ~SSHSession() = default;

    bool initiateConnection()
    {
        try
        {
            session.connect();
            session.userauthPublickeyAuto();
            return true;
        }
        catch (ssh::SshException e)
        {

            return false;
        }
    }

private:
    ssh::Session session;
};

#endif