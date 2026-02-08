const express = require('express');
const path = require('path');
const app = express();
const PORT = 3000;

// Serve static files from the current directory
app.use(express.static('.'));

// Handle all routes by serving the index.html file
app.get(/.*/, (req, res) => {
    res.sendFile(path.join(__dirname, 'index.html'));
});

app.listen(PORT, 'localhost', () => {
    console.log(``);
    console.log(`*************************************************************`);
    console.log(`*                                                           *`);
    console.log(`*    YOUR BOOK IS NOW ACCESSIBLE AT: http://localhost:3000   *`);
    console.log(`*                                                           *`);
    console.log(`*    The beautiful UI with all chapters and links           *`);
    console.log(`*    is now displayed in your browser                       *`);
    console.log(`*                                                           *`);
    console.log(`*************************************************************`);
    console.log(``);
});