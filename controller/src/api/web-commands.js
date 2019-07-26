export default class HTTP { 

    static doPost(host, message) { 
        console.log("Attempting to post: " + message);
        console.log("\nTo Host: " + host);

        return fetch(
            host,
            {
                headers: {"Content-Type":"text/plain; charset=utf-8"},
                method: 'POST',
                headers:{
                    Accept: 'application/json',
                },
                body: message
            }
        )
        .catch((error) => {
            console.log(error);
        });
    }
}